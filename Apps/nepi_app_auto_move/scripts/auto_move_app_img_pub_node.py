#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
#
# License: nepi applications are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#


import os
import copy
import math
import threading

import numpy as np
import cv2

from sensor_msgs.msg import Image

from nepi_interfaces.msg import ImageStatus
from nepi_interfaces.msg import Targets

from nepi_app_auto_move.msg import NepiAppAutoMoveStatus

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF
from nepi_api.data_if import ColorImageIF


# The obstacle overlay layer is optional, exactly as in auto_move_if.py: without
# nepi_app_obstacles installed there is no Obstacles message to import, the
# subscriber is never created and the layer draws nothing.
try:
    from nepi_app_obstacles.msg import Obstacles
    HAS_OBSTACLES_MSG = True
except ImportError:
    Obstacles = None
    HAS_OBSTACLES_MSG = False


WATCHDOG_DELAY = 60
WATCHDOG_TIMEOUT = 3

# How much source frame history the aligned-image lookup can search. Two seconds
# is well past any observed depth map latency, and the count cap keeps a fast
# source from holding more than a handful of frames. Raw ROS Image msgs are
# buffered undecoded, so a frame that never gets published costs only its bytes.
MAX_IMG_BUFFER_SEC = 2.0
MAX_IMG_BUFFER_LEN = 10

# Overlay colours, BGR
OBSTACLE_BOX_COLOR = (0, 128, 255)
OBJECT_BOX_COLOR = (0, 255, 0)
TARGET_MARKER_COLOR = (0, 255, 255)
CROSSHAIR_COLOR = (0, 0, 255)

# Label font, hoisted out of the draw loop. Nothing about it varies per frame.
OVERLAY_FONT = cv2.FONT_HERSHEY_DUPLEX
OVERLAY_FONT_COLOR = (255, 255, 255)
OVERLAY_LABEL_BOX_COLOR = (0, 0, 0)
OVERLAY_LINE_TYPE = cv2.LINE_AA

# How many distinct frame sizes the per-size render caches keep before they are
# dropped and rebuilt. A source's resolution changes rarely -- the cap only
# exists so a source that walks its resolution control cannot grow them without
# bound.
MAX_RENDER_CACHE_LEN = 8

# Crosshair geometry, as a fraction of the frame's larger dimension.
CROSSHAIR_SIZE_RATIO = 0.04

# Fallback until the first parent status arrives. 0.0 fully opaque through 1.0
# invisible.
DEFAULT_DEPTH_MAP_TRANSPARENCY = 0.5

# Publish rate ceiling for the overlay. The app publishes no rate control of its
# own -- there is one overlay and one operator watching it -- so the cap is
# fixed here rather than carried on the status message.
MAX_IMAGE_PUB_RATE_HZ = 10.0

# Image data product this node publishes. Must match OVERLAY_IMG_PRODUCT in
# auto_move_if.py; that class only reports where this topic will appear, this
# node owns the publisher.
OVERLAY_IMG_PRODUCT = 'auto_move_image'


class AutoMoveImgPub:

    OVERLAY_IMG_DATA_PRODUCT = OVERLAY_IMG_PRODUCT

    DATA_PRODUCTS = [OVERLAY_IMG_DATA_PRODUCT]

    # Never subscribe to our own overlay output as an input image source; skip
    # this product basename even if a stale selection names it.
    OUTPUT_IMG_PRODUCTS = [OVERLAY_IMG_DATA_PRODUCT]

    node_if = None
    save_data_if = None

    # Publishers and IFs for the one selected source, replaced whole when the
    # selection changes.
    img_node_dict = dict()
    img_node_lock = threading.Lock()

    source_info_dict = None
    info_lock = threading.Lock()

    # Recent source frames, newest last, as (stamp, raw Image msg). Held outside
    # source_info_dict on purpose: copying full frames on every status tick
    # would cost more than the alignment saves.
    img_buffer = []
    img_buffer_lock = threading.Lock()

    # Newest arrived source frame, as (stamp, raw Image msg). One slot,
    # overwritten rather than appended: a frame the render never got to is
    # dropped here, so the render can only ever be one frame behind the source
    # instead of falling progressively further behind.
    render_slot = None
    render_slot_lock = threading.Lock()

    # One cycle's worth of drawable data, replaced whole and never mutated after
    # it is stored. The render path takes one reference and gets the depth map
    # render, the three overlay lists and the stamp they belong to together --
    # reading those fields separately is what lets a cycle boundary land in the
    # middle of a render and pair one cycle's boxes with another cycle's frame.
    results_dict = None
    results_lock = threading.Lock()

    # Per-frame-size render constants, built once per size instead of once per
    # frame. Written and read only by the single render thread.
    font_dims_cache = dict()

    last_status_time = None
    watchdog_timeout = None

    data_products = DATA_PRODUCTS

    # Everything below arrives on the parent app's status message.
    image_topic = 'None'
    depth_map_topic = ''
    depth_map_image_topic = ''

    depth_map_transparency = DEFAULT_DEPTH_MAP_TRANSPARENCY
    show_depth_map_enabled = True
    show_objects_enabled = True
    show_targets_enabled = True
    show_obstacles_enabled = True
    show_crosshair_enabled = True

    has_click = False
    click_x_ratio = 0.0
    click_y_ratio = 0.0

    overlay_labels = True
    overlay_range_bearing = True

    DEFAULT_NODE_NAME = "auto_move_img_pub"  # Can be overwritten by launch command

    def __init__(self):
        ####  NODE INIT SETUP ####
        nepi_sdk.init_node(name = self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################
        # Create Msg Class
        self.msg_if = MsgIF()
        self.msg_if.pub_info("Starting Node Initialization Processes")

        ##############################
        # Init Class Variables

        # This node is launched by the parent app node as
        # <parent node namespace>_img_pub, so stripping the suffix recovers the
        # parent's namespace.
        self.app_namespace = self.node_namespace.replace("_img_pub", "")
        self.all_namespace = nepi_sdk.create_namespace(self.base_namespace, 'all')
        self.msg_if.pub_info("Starting with App Namespace: " + str(self.app_namespace))

        self.results_dict = self.createResultDict()

        ##############################
        # Create NodeClassIF Class

        # Configs Dict ########################
        # This node holds no configuration of its own -- every setting it
        # honors arrives on the parent's status message.
        self.CONFIGS_DICT = None

        # Params Config Dict ####################
        self.PARAMS_DICT = None

        # Services Config Dict ####################
        self.SRVS_DICT = None

        # Pubs Config Dict ####################
        self.PUBS_DICT = None

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'auto_move_status_sub': {
                'msg': NepiAppAutoMoveStatus,
                'namespace': self.app_namespace,
                'topic': 'status',
                'qsize': 10,
                'callback': self.statusCb,
                'callback_args': ()
            },

            'auto_move_all_targets_sub': {
                'msg': Targets,
                'namespace': self.all_namespace,
                'topic': 'targets',
                'qsize': 1,
                'callback': self.targetsCb,
                'callback_args': ()
            },
        }

        if HAS_OBSTACLES_MSG == True:
            self.SUBS_DICT['auto_move_all_obstacles_sub'] = {
                'msg': Obstacles,
                'namespace': self.all_namespace,
                'topic': 'obstacles',
                'qsize': 1,
                'callback': self.obstaclesCb,
                'callback_args': ()
            }

        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CONFIGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        msg_if = self.msg_if
                        )
        self.node_if.wait_for_ready()

        ###############################
        # Create System IFs

        # Setup Save Data IF. pub_status is False because the parent node owns
        # the save_data status topic on this same namespace; this instance is
        # only here so the image data product honors the same save commands.
        factory_data_rates = {}
        for d in self.data_products:
            factory_data_rates[d] = [1.0, 0.0, 100]

        self.save_data_if = SaveDataIF(namespace = self.app_namespace,
                        data_products = self.data_products,
                        pub_status = False,
                        factory_rate_dict = factory_data_rates,
                        msg_if = self.msg_if,
                        node_if = self.node_if
                        )
        nepi_sdk.sleep(1)

        ##########################
        # Complete Initialization

        # Start Timer Processes
        nepi_sdk.start_timer_process((1.0), self.updaterCb, oneshot = True)
        nepi_sdk.start_timer_process((1.0), self.renderCb, oneshot = True)
        self.last_status_time = nepi_utils.get_time()
        nepi_sdk.start_timer_process(1, self.watchdogCb, oneshot = True)
        nepi_sdk.on_shutdown(self.shutdownCb)

        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Spin forever
        nepi_sdk.spin()
        #########################################################


    ###############################
    # Class Private Methods
    ###############################

    def getSourceInfo(self):
        self.info_lock.acquire()
        source_info_dict = copy.deepcopy(self.source_info_dict)
        self.info_lock.release()
        return source_info_dict

    def getActiveSourceTopic(self):
        source_info_dict = self.getSourceInfo()
        if source_info_dict is None:
            return None
        if source_info_dict.get('active', False) == False:
            return None
        return source_info_dict.get('source_topic', None)

    def createSourceInfoDict(self, source_topic, pub_namespace):
        info_dict = dict()
        info_dict['source_topic'] = source_topic
        info_dict['active'] = True
        info_dict['pub_namespace'] = pub_namespace
        info_dict['img_connected'] = False
        info_dict['img_published'] = False
        info_dict['status_dict'] = None
        info_dict['last_img_time'] = 0
        info_dict['get_latency_time'] = 0
        info_dict['pub_latency_time'] = 0
        info_dict['process_time'] = 0

        # Cached answer to "does the published product still want data",
        # refreshed by updaterCb. imageCb reads this instead of calling
        # needsImgCheck, because that takes img_node_lock and the render path
        # holds that lock -- a subscriber callback that can block on a render is
        # a subscriber callback that loses the frames the alignment lookup
        # needs. The underlying IF flag is itself only refreshed once a second,
        # so a cached copy loses nothing.
        info_dict['needs_img'] = False
        return info_dict

    ###############.########################
    # Source frame buffer

    def bufferImgMsg(self, img_msg, timestamp):
        # Every arriving frame goes in, not just the ones a publish cycle lands
        # on, because the aligned lookup can only find the frame the depth map
        # render came from if that frame was kept.
        self.img_buffer_lock.acquire()
        self.img_buffer.append((timestamp, img_msg))
        # Age first, against the newest stamp, then the hard count cap.
        while len(self.img_buffer) > 1 and (timestamp - self.img_buffer[0][0]) > MAX_IMG_BUFFER_SEC:
            self.img_buffer.pop(0)
        while len(self.img_buffer) > MAX_IMG_BUFFER_LEN:
            self.img_buffer.pop(0)
        self.img_buffer_lock.release()

    def getAlignedImgMsg(self, target_stamp):
        # The buffered frame closest in time to the frame the overlay data was
        # derived from. Nearest rather than exact match because the colour image
        # and the depth map are separate products and their stamps are only
        # equal when the driver publishes them from one capture -- nearest gives
        # the exact frame when they agree and the best available frame when they
        # do not. Returns None when there is nothing buffered or no data stamp
        # to align to yet, and the caller falls back to the frame in hand.
        if target_stamp is None or float(target_stamp) <= 0:
            return None
        target_stamp = float(target_stamp)
        best_msg = None
        best_delta = None
        self.img_buffer_lock.acquire()
        for entry in self.img_buffer:
            delta = abs(entry[0] - target_stamp)
            if best_delta is None or delta < best_delta:
                best_delta = delta
                best_msg = entry[1]
        self.img_buffer_lock.release()
        return best_msg

    def clearImgBuffer(self):
        self.img_buffer_lock.acquire()
        self.img_buffer = []
        self.img_buffer_lock.release()

    ###############.########################
    # Per-cycle result snapshots

    def createResultDict(self):
        return {
            # The stamp of the depth map render the overlay lists belong to --
            # what the render aligns the published frame against.
            'depth_map_stamp': 0,
            'depth_map_image': None,
            'objects_list': [],
            'targets_list': [],
            'obstacles_list': [],
        }

    def getResult(self):
        # One reference, one read, no copy. Entries are immutable once stored,
        # so the depth map render does not have to be copied out of the way of
        # the next cycle's writer.
        result_dict = self.results_dict
        if result_dict is None:
            return self.createResultDict()
        return result_dict

    def setResult(self, result_dict):
        self.results_lock.acquire()
        self.results_dict = result_dict
        self.results_lock.release()

    def clearResult(self):
        self.results_lock.acquire()
        self.results_dict = self.createResultDict()
        self.results_lock.release()

    ###############.########################
    # Render handoff

    def setRenderSlot(self, timestamp, img_msg):
        self.render_slot_lock.acquire()
        self.render_slot = (timestamp, img_msg)
        self.render_slot_lock.release()

    def popRenderSlot(self):
        # Taking the frame out is what makes this a slot and not a queue: if the
        # render is slower than the source, the frames that arrive in between
        # overwrite each other and only the newest is ever drawn.
        self.render_slot_lock.acquire()
        slot = self.render_slot
        self.render_slot = None
        self.render_slot_lock.release()
        return slot

    def clearRenderSlot(self):
        self.render_slot_lock.acquire()
        self.render_slot = None
        self.render_slot_lock.release()

    ###############.########################
    # Source subscription

    def updaterCb(self, timer):
        image_topic = self.image_topic
        active_topic = self.getActiveSourceTopic()

        want_topic = None
        if image_topic is not None and image_topic != 'None' and image_topic != '':
            if os.path.basename(image_topic) not in self.OUTPUT_IMG_PRODUCTS:
                found = nepi_sdk.find_topic(image_topic, exact = True)
                if found != '':
                    want_topic = found

        if active_topic is not None and active_topic != want_topic:
            self.msg_if.pub_info('Will unsubscribe from image topic: ' + str(active_topic))
            self.unsubscribeImgTopic()
            active_topic = None

        if want_topic is not None and active_topic is None:
            self.msg_if.pub_info('Will subscribe to image topic: ' + str(want_topic))
            self.subscribeImgTopic(want_topic)

        # Refresh the cached needs-data answer imageCb reads. Done here, on a
        # timer, because the IF flag it comes from is itself refreshed once a
        # second, and because reading it takes img_node_lock.
        needs_img = self.needsImgCheck()
        self.info_lock.acquire()
        if self.source_info_dict is not None:
            self.source_info_dict['needs_img'] = needs_img
        self.info_lock.release()

        nepi_sdk.start_timer_process((1), self.updaterCb, oneshot = True)

    def watchdogCb(self, timer):
        cur_time = nepi_utils.get_time()
        elapsed = cur_time - self.last_status_time
        if self.watchdog_timeout is None:
            self.watchdog_timeout = WATCHDOG_TIMEOUT
            nepi_sdk.sleep(WATCHDOG_DELAY)
        else:
            if elapsed > WATCHDOG_TIMEOUT:
                msg = "Lost connection to parent node status msg.  Shutting down"
                self.msg_if.pub_warn(msg)
                nepi_sdk.signal_shutdown(msg)
                return

        nepi_sdk.start_timer_process(1, self.watchdogCb, oneshot = True)

    def subscribeImgTopic(self, source_topic):
        if source_topic == "None" or source_topic == "":
            return False

        pub_namespace = os.path.dirname(source_topic)
        img_pub_topic = os.path.join(pub_namespace, self.OVERLAY_IMG_DATA_PRODUCT)

        self.msg_if.pub_info('Publishing image ' + source_topic + ' on namespace: ' + img_pub_topic)

        img_pub = nepi_sdk.create_publisher(img_pub_topic, Image, queue_size = 1)
        nepi_sdk.sleep(1)
        img_sub = nepi_sdk.create_subscriber(source_topic, Image, self.imageCb, queue_size = 1)
        img_status_topic = nepi_sdk.create_namespace(source_topic, 'status')
        img_status_sub = nepi_sdk.create_subscriber(img_status_topic, ImageStatus, self.imageStatusCb, queue_size = 1)

        # The depth map render is a plain Image subscriber rather than a data IF:
        # this node only needs the colourized frame to blend, and the topic is
        # reported by the parent rather than selected here.
        depth_map_image_sub = None
        depth_map_image_topic = self.depth_map_image_topic
        if depth_map_image_topic is not None and depth_map_image_topic != '':
            depth_map_image_sub = nepi_sdk.create_subscriber(depth_map_image_topic, Image,
                                                             self.depthMapImageCb, queue_size = 1)
            self.msg_if.pub_info('Subscribing to depth map image: ' + str(depth_map_image_topic))

        img_if = ColorImageIF(namespace = pub_namespace,
                        data_product = self.OVERLAY_IMG_DATA_PRODUCT,
                        data_source_description = 'image',
                        data_ref_description = 'image',
                        perspective = 'pov',
                        save_data_if = self.save_data_if,
                        init_overlay_text_list = [],
                        live_adjustments_disabled = True,
                        aspect_adjustment_disabled = True,
                        msg_if = self.msg_if)

        self.img_node_lock.acquire()
        self.img_node_dict = {
                            'img_sub': img_sub,
                            'img_status_sub': img_status_sub,
                            'depth_map_image_sub': depth_map_image_sub,
                            'depth_map_image_topic': depth_map_image_topic,
                            'img_pub': img_pub,
                            'img_if': img_if,
                            }
        self.img_node_lock.release()

        self.info_lock.acquire()
        self.source_info_dict = self.createSourceInfoDict(source_topic, pub_namespace)
        self.info_lock.release()

        return True

    def unsubscribeImgTopic(self):
        self.img_node_lock.acquire()
        img_node_dict = self.img_node_dict
        self.img_node_dict = dict()
        self.img_node_lock.release()

        for key in ['img_sub', 'img_status_sub', 'depth_map_image_sub', 'img_pub']:
            handle = img_node_dict.get(key, None)
            if handle is not None:
                try:
                    handle.unregister()
                except Exception:
                    pass
        img_if = img_node_dict.get('img_if', None)
        if img_if is not None:
            try:
                img_if.unregister_pubs()
            except Exception:
                pass

        self.info_lock.acquire()
        self.source_info_dict = None
        self.info_lock.release()

        # Nothing in the buffer survives an unsubscribe -- on resubscribe those
        # frames would be arbitrarily old and could win an alignment lookup. The
        # pending render frame and the last result snapshot go for the same
        # reason: both describe a stream this node is no longer following.
        self.clearImgBuffer()
        self.clearRenderSlot()
        self.clearResult()

        return True

    def needsImgCheck(self):
        # The IF's needs_data is a level flag its own timer refreshes from
        # subscriber and save state, so polling it is free.
        needs_img = False
        self.img_node_lock.acquire()
        img_if = self.img_node_dict.get('img_if', None)
        self.img_node_lock.release()
        if img_if is not None:
            try:
                needs_img = (img_if.needs_data_check() == True)
            except Exception:
                needs_img = False
        return needs_img

    ###############.########################
    # Data Callbacks

    def imageStatusCb(self, status_msg):
        self.info_lock.acquire()
        if self.source_info_dict is not None:
            status_dict = nepi_sdk.convert_msg2dict(status_msg)
            if self.source_info_dict['status_dict'] is None:
                self.msg_if.pub_info('Connected to image status topic')
            self.source_info_dict['status_dict'] = status_dict
        self.info_lock.release()

    def imageCb(self, image_msg):
        info_dict = self.source_info_dict
        if info_dict is None:
            return

        if info_dict['img_connected'] == False:
            self.msg_if.pub_info('Connected to image topic: ' + str(info_dict['source_topic']))
        info_dict['img_connected'] = True

        # This callback is deliberately cheap. The overlay render runs on
        # renderCb, not here: the transport drops every frame that arrives while
        # a callback is busy (qsize 1), so rendering here would mean the frames
        # captured during a render never reach the alignment buffer -- and the
        # frame the depth map render was actually derived from would usually not
        # be there to be found.
        if info_dict['needs_img'] == False:
            return

        timestamp = float(image_msg.header.stamp.to_sec())
        self.bufferImgMsg(image_msg, timestamp)
        info_dict['get_latency_time'] = (nepi_utils.get_time() - timestamp)
        self.setRenderSlot(timestamp, image_msg)

    def depthMapImageCb(self, image_msg):
        # The colourized depth map render, kept as the cv2 frame the blend
        # wants. Stored as a whole replacement result entry so the render path
        # never reads a half-updated one.
        cv2_img = None
        try:
            cv2_img = nepi_img.rosimg_to_cv2img(image_msg)
        except Exception as e:
            self.msg_if.pub_warn("Failed to convert depth map image: " + str(e), throttle_s = 5.0)
            return

        result_dict = dict(self.getResult())
        result_dict['depth_map_image'] = cv2_img
        result_dict['depth_map_stamp'] = float(image_msg.header.stamp.to_sec())
        self.setResult(result_dict)

    def targetsCb(self, msg):
        # One fan-out, two overlays. Object boxes used to be drawn from
        # <base>/all/detections and target markers from <base>/all/targets;
        # only targets survives, so both overlay lists are filled from this one
        # message. Keeping them as separate result keys keeps the two RUI
        # toggles (show_objects_enabled, show_targets_enabled) independent, and
        # the box and the centre marker are still two different drawings of the
        # same thing: a box with a dot in it, which is how it has always looked.
        if self.image_topic == 'None' or msg.data_header.source_topic != self.image_topic:
            return
        targets_dict = nepi_sdk.convert_msg2dict(msg)
        targets_dict_list = targets_dict['targets']
        result_dict = dict(self.getResult())
        result_dict['targets_list'] = targets_dict_list
        result_dict['objects_list'] = list(targets_dict_list)
        self.setResult(result_dict)

    def obstaclesCb(self, msg):
        # Obstacles match on the DEPTH MAP topic, not the image topic: an
        # obstacles process consumes depth maps, so that is what its
        # source_topic names.
        if self.depth_map_topic == '' or msg.data_header.source_topic != self.depth_map_topic:
            return
        obstacles_dict = nepi_sdk.convert_msg2dict(msg)
        obstacles_dict_list = obstacles_dict['obstacles']
        result_dict = dict(self.getResult())
        result_dict['obstacles_list'] = obstacles_dict_list
        self.setResult(result_dict)

    def statusCb(self, msg):
        self.last_status_time = nepi_utils.get_time()

        last_image_topic = self.image_topic
        last_depth_map_image_topic = self.depth_map_image_topic

        self.image_topic = msg.image_topic
        self.depth_map_topic = msg.depth_map_topic
        self.depth_map_image_topic = msg.depth_map_image_topic

        self.depth_map_transparency = msg.depth_map_transparency
        self.show_depth_map_enabled = msg.show_depth_map_enabled
        self.show_objects_enabled = msg.show_objects_enabled
        self.show_targets_enabled = msg.show_targets_enabled
        self.show_obstacles_enabled = msg.show_obstacles_enabled
        self.show_crosshair_enabled = msg.show_crosshair_enabled

        self.has_click = msg.has_click
        self.click_x_ratio = msg.last_click_x_ratio
        self.click_y_ratio = msg.last_click_y_ratio

        if last_image_topic != self.image_topic:
            self.msg_if.pub_info("Updating selected image topic: " + str(self.image_topic))

        # A depth map image topic that changes under a source that did not is
        # re-subscribed on its own, so the blend follows the parent's resolution
        # without tearing down the source subscription.
        if last_depth_map_image_topic != self.depth_map_image_topic:
            self.updateDepthMapImageSub()

    def updateDepthMapImageSub(self):
        self.img_node_lock.acquire()
        has_source = ('img_sub' in self.img_node_dict.keys())
        old_sub = self.img_node_dict.get('depth_map_image_sub', None)
        self.img_node_lock.release()

        if has_source == False:
            return

        if old_sub is not None:
            try:
                old_sub.unregister()
            except Exception:
                pass

        new_sub = None
        depth_map_image_topic = self.depth_map_image_topic
        if depth_map_image_topic is not None and depth_map_image_topic != '':
            new_sub = nepi_sdk.create_subscriber(depth_map_image_topic, Image,
                                                 self.depthMapImageCb, queue_size = 1)
            self.msg_if.pub_info('Subscribing to depth map image: ' + str(depth_map_image_topic))

        self.img_node_lock.acquire()
        self.img_node_dict['depth_map_image_sub'] = new_sub
        self.img_node_dict['depth_map_image_topic'] = depth_map_image_topic
        self.img_node_lock.release()

        result_dict = dict(self.getResult())
        result_dict['depth_map_image'] = None
        result_dict['depth_map_stamp'] = 0
        self.setResult(result_dict)

    ###############.########################
    # Render

    def renderCb(self, timer):
        start_time = nepi_utils.get_time()
        delay_time = float(1) / MAX_IMAGE_PUB_RATE_HZ

        info_dict = self.source_info_dict
        if info_dict is None:
            self.clearRenderSlot()
        else:
            # Rate gate before the slot is emptied, not after: a source slower
            # than this timer would otherwise have its only frame thrown away by
            # a tick that was not allowed to publish it.
            current_time = nepi_utils.get_time()
            if round((current_time - info_dict['last_img_time']), 3) > delay_time:
                slot = self.popRenderSlot()
                if slot is not None:
                    info_dict['last_img_time'] = current_time
                    self.renderSourceFrame(slot[1])

        # Ticks at half the publish period so jitter cannot cost every other
        # publish, minus whatever this pass already spent, floored the way the
        # parent's process loop floors its own oneshot chain.
        cycle_time = nepi_utils.get_time() - start_time
        next_delay = (delay_time / 2.0) - cycle_time
        if next_delay < 0.01:
            next_delay = 0.01
        nepi_sdk.start_timer_process((next_delay), self.renderCb, oneshot = True)

    def renderSourceFrame(self, img_msg):
        start_time = nepi_utils.get_time()

        # One snapshot of one cycle: the depth map render and the three overlay
        # lists all came from the same stored entry, and nothing can replace
        # them underneath this render.
        result_dict = self.getResult()

        # Align the published frame with the depth map render it is being
        # blended with, by stamp, so the blend and the scene belong to the same
        # instant at any source rate. With no depth map render to align to, the
        # newest frame is used as it stands.
        use_img_msg = img_msg
        aligned_img_msg = self.getAlignedImgMsg(result_dict['depth_map_stamp'])
        if aligned_img_msg is not None:
            use_img_msg = aligned_img_msg

        # The stamp and frame of the frame actually drawn on, not of the frame
        # that triggered the cycle -- otherwise an aligned publish claims to be
        # fresher than it is and every downstream latency reads low.
        use_timestamp = float(use_img_msg.header.stamp.to_sec())
        use_frame_id = use_img_msg.header.frame_id
        try:
            use_cv2_img = nepi_img.rosimg_to_cv2img(use_img_msg)
        except Exception as e:
            self.msg_if.pub_warn("Failed to convert source image: " + str(e), throttle_s = 5.0)
            return

        self.processOverlayImage(use_cv2_img,
                                 result_dict,
                                 timestamp = use_timestamp,
                                 frame_id = use_frame_id)

        info_dict = self.source_info_dict
        if info_dict is not None:
            # pub_latency_time is how old the published pixels are; process_time
            # is what this render cost.
            info_dict['pub_latency_time'] = (nepi_utils.get_time() - use_timestamp)
            info_dict['process_time'] = (nepi_utils.get_time() - start_time)

    def processOverlayImage(self, cv2_img, result_dict, timestamp = None, frame_id = ''):
        info_dict = self.source_info_dict
        if info_dict is None:
            return False

        # Nobody watching means no render at all -- not a render thrown away at
        # the publish gate.
        if self.needsImgCheck() == False:
            return False

        status_dict = info_dict['status_dict']
        if status_dict is not None:
            width_deg = status_dict.get('width_deg', 100)
            height_deg = status_dict.get('height_deg', 70)
        else:
            width_deg = 100
            height_deg = 70

        if cv2_img is None:
            return False

        # cv_bridge hands back an array backed by the source message's own bytes,
        # and that message is still in the alignment buffer, so the drawing gets
        # its own buffer. One copy for the whole cycle instead of one per
        # overlay step.
        overlay_img = cv2_img.copy()

        # Layer order is fixed: the depth map blend is a background wash, and
        # every marker drawn after it stays legible on top of it.
        if self.show_depth_map_enabled == True:
            self.applyDepthMapOverlay(overlay_img,
                                      result_dict['depth_map_image'],
                                      self.getBlendRatio(self.depth_map_transparency))

        if self.show_obstacles_enabled == True:
            self.applyBoxesOverlay(result_dict['obstacles_list'], overlay_img, OBSTACLE_BOX_COLOR)

        if self.show_objects_enabled == True:
            self.applyBoxesOverlay(result_dict['objects_list'], overlay_img, OBJECT_BOX_COLOR)

        if self.show_targets_enabled == True:
            self.applyTargetsOverlay(result_dict['targets_list'], overlay_img, TARGET_MARKER_COLOR)

        if self.show_crosshair_enabled == True and self.has_click == True:
            self.applyCrosshairOverlay(overlay_img, self.click_x_ratio, self.click_y_ratio, CROSSHAIR_COLOR)

        self.publishImgData(overlay_img,
                            width_deg = width_deg,
                            height_deg = height_deg,
                            timestamp = timestamp,
                            frame_id = frame_id,
                            add_overlay_text_list = [])

        if info_dict['img_published'] == False:
            namespace = info_dict['pub_namespace']
            self.msg_if.pub_info('Published image topic: ' + os.path.join(namespace, self.OVERLAY_IMG_DATA_PRODUCT))
        info_dict['img_published'] = True

        return True

    def publishImgData(self, cv2_img, encoding = "bgr8", timestamp = None,
                        frame_id = '',
                        width_deg = 100,
                        height_deg = 70,
                        add_overlay_text_list = []):
        # The lock covers looking the publishers up, not publishing through
        # them. Held across the publish it would serialize the whole encode, and
        # any thread that so much as asked whether the product needed data would
        # wait behind that. A publisher torn down by unsubscribeImgTopic between
        # the lookup and the publish raises, which is what the try/except is for.
        self.img_node_lock.acquire()
        img_if = self.img_node_dict.get('img_if', None)
        img_pub = self.img_node_dict.get('img_pub', None)
        self.img_node_lock.release()

        try:
            if img_if is None or img_if.ready == False:
                img_msg = nepi_img.cv2img_to_rosimg(cv2_img, encoding = encoding)
                # Stamp the raw fallback publish the same way the IF path stamps
                # its own: with the source frame this image was drawn on. Left
                # unset it goes out with a zero stamp and no frame, and nothing
                # downstream can time-align it.
                img_msg.header = nepi_sdk.create_header_msg(time_sec = timestamp, frame_id = frame_id)
                nepi_sdk.publish_pub(img_pub, img_msg)
            else:
                img_if.publish_cv2_img(cv2_img,
                                    encoding = encoding,
                                    timestamp = timestamp,
                                    width_deg = width_deg,
                                    height_deg = height_deg,
                                    add_overlay_text_list = add_overlay_text_list
                                    )
        except Exception as e:
            self.msg_if.pub_warn("Failed to publish overlay image: " + str(e), throttle_s = 5.0)

    def getBlendRatio(self, transparency):
        # The wire value is transparency, the way the RUI slider labels it;
        # addWeighted wants the overlay's share, which is its complement.
        try:
            transparency = float(transparency)
        except (TypeError, ValueError):
            return 0.5
        if transparency < 0.0:
            transparency = 0.0
        elif transparency > 1.0:
            transparency = 1.0
        return 1.0 - transparency

    def getOverlayFontDims(self, cv2_img):
        # Label metrics depend only on the frame size, so they are computed once
        # per size rather than once per frame.
        cv2_shape = cv2_img.shape
        key = (cv2_shape[0], cv2_shape[1])
        font_dims = self.font_dims_cache.get(key, None)
        if font_dims is None:
            img_height = cv2_shape[0]
            img_width = cv2_shape[1]
            scale = 1.5e-3 - 0.1e-3 * math.ceil(max([img_height, img_width]) / 700)
            [font_scale, font_thickness] = nepi_img.get_optimal_font_dims(cv2_img, font_scale = scale, thickness_scale = scale)
            line_thickness = 1 + math.ceil(max([img_height, img_width]) / 2000)
            font_dims = [font_scale, font_thickness, line_thickness]
            if len(self.font_dims_cache) >= MAX_RENDER_CACHE_LEN:
                self.font_dims_cache.clear()
            self.font_dims_cache[key] = font_dims
        return font_dims

    def applyDepthMapOverlay(self, cv2_img, cv2_map, blend_ratio):
        # Blend the colourized depth map render into cv2_img. The render is a
        # bgr8 image the depth map producer already colourized, so this is a
        # whole-frame blend rather than a masked one -- there is no NaN mask on
        # a rendered image to segment with.
        #
        # Writes in place. The caller owns cv2_img for the whole cycle, so the
        # blend costs one addWeighted instead of a blend plus a frame copy.
        try:
            if cv2_map is None:
                return
            if cv2_map.shape[:2] != cv2_img.shape[:2]:
                cv2_map = cv2.resize(cv2_map, (cv2_img.shape[1], cv2_img.shape[0]))
            if len(cv2_map.shape) == 2:
                cv2_map = cv2.cvtColor(cv2_map, cv2.COLOR_GRAY2BGR)
            blended = cv2.addWeighted(cv2_img, (1.0 - blend_ratio), cv2_map, blend_ratio, 0)
            np.copyto(cv2_img, blended)
        except Exception as e:
            self.msg_if.pub_warn("Failed to apply depth map overlay: " + str(e), throttle_s = 5.0)

    def applyBoxesOverlay(self, boxes_dict_list, cv2_img, default_color):
        # Draws in place, same as applyDepthMapOverlay and for the same reason.
        if boxes_dict_list is None or len(boxes_dict_list) == 0:
            return cv2_img

        cv2_shape = cv2_img.shape
        img_size = cv2_shape[:2]

        font = OVERLAY_FONT
        [fontScale, fontThickness, line_thickness] = self.getOverlayFontDims(cv2_img)
        fontColor = OVERLAY_FONT_COLOR
        lineType = OVERLAY_LINE_TYPE

        for box_dict in boxes_dict_list:
            class_name = box_dict['name']
            xmin = box_dict['xmin_pixel']
            ymin = box_dict['ymin_pixel']
            xmax = box_dict['xmax_pixel']
            ymax = box_dict['ymax_pixel']

            if xmin <= 0:
                xmin = 5
            if ymin <= 0:
                ymin = 5
            if xmax >= img_size[1]:
                xmax = img_size[1] - 5
            if ymax >= img_size[0]:
                ymax = img_size[0] - 5

            bot_left_box = (xmin, ymin)
            top_right_box = (xmax, ymax)

            success = False
            try:
                cv2.rectangle(cv2_img, bot_left_box, top_right_box, default_color, thickness = line_thickness)
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed to create bounding box rectangle: " + str(e), throttle_s = 5.0)

            if success == False:
                continue

            ## Overlay Text
            overlay_text = ""
            if self.overlay_labels:
                overlay_text = overlay_text + str(class_name) + " "
            if self.overlay_range_bearing:
                rb_text = ''
                if box_dict['range_m'] != -999 and box_dict['range_m'] != '':
                    rb_text = rb_text + str(round(box_dict['range_m'], 1)) + 'm :'
                if box_dict['azimuth_deg'] != -999 and box_dict['elevation_deg'] != -999:
                    rb_text = rb_text + str(round(box_dict['azimuth_deg'], 1)) + 'deg '
                    rb_text = rb_text + str(round(box_dict['elevation_deg'], 1)) + 'deg '
                if len(rb_text) > 0:
                    overlay_text = overlay_text + rb_text

            if len(overlay_text) == 0:
                continue

            text_size = cv2.getTextSize(overlay_text, font, fontScale, fontThickness)
            line_height = text_size[0][1]
            line_width = text_size[0][0]
            x_padding = int(line_height * 0.4)
            y_padding = int(line_height * 0.4)

            center = bot_left_box[0] + int((top_right_box[0] - bot_left_box[0]) / 2)
            bot_left_text = (center + x_padding, ymin - (line_thickness * 2) - y_padding)
            text_bot_left_box = (center - x_padding, bot_left_text[1] + y_padding)
            text_top_right_box = (center + line_width + x_padding, bot_left_text[1] - line_height - y_padding)
            box_color = OVERLAY_LABEL_BOX_COLOR

            try:
                cv2.rectangle(cv2_img, text_bot_left_box, text_top_right_box, box_color, -1)
                cv2.putText(cv2_img, overlay_text,
                    bot_left_text,
                    font,
                    fontScale,
                    fontColor,
                    fontThickness,
                    lineType)
            except Exception as e:
                self.msg_if.pub_warn("Failed to apply overlay label text: " + str(e), throttle_s = 5.0)

        return cv2_img

    def applyTargetsOverlay(self, targets_dict_list, cv2_img, color):

        if targets_dict_list is None or len(targets_dict_list) == 0:
            return cv2_img

        [fontScale, fontThickness, line_thickness] = self.getOverlayFontDims(cv2_img)
        height_px = cv2_img.shape[0]
        width_px = cv2_img.shape[1]
        marker_px = max(int(max(width_px, height_px) * 0.015), 4)

        for target_dict in targets_dict_list:
            try:
                x_center = int((target_dict['xmin_pixel'] + target_dict['xmax_pixel']) / 2)
                y_center = int((target_dict['ymin_pixel'] + target_dict['ymax_pixel']) / 2)
                if x_center < 0 or y_center < 0 or x_center >= width_px or y_center >= height_px:
                    continue
                cv2.circle(cv2_img, (x_center, y_center), marker_px, color, thickness = line_thickness)
                cv2.line(cv2_img, (x_center - marker_px * 2, y_center), (x_center - marker_px, y_center), color, line_thickness)
                cv2.line(cv2_img, (x_center + marker_px, y_center), (x_center + marker_px * 2, y_center), color, line_thickness)
                cv2.line(cv2_img, (x_center, y_center - marker_px * 2), (x_center, y_center - marker_px), color, line_thickness)
                cv2.line(cv2_img, (x_center, y_center + marker_px), (x_center, y_center + marker_px * 2), color, line_thickness)

                name = str(target_dict.get('name', ''))
                if len(name) > 0:
                    cv2.putText(cv2_img, name,
                                (x_center + marker_px * 2, y_center - marker_px),
                                OVERLAY_FONT,
                                fontScale,
                                OVERLAY_FONT_COLOR,
                                fontThickness,
                                OVERLAY_LINE_TYPE)
            except Exception as e:
                self.msg_if.pub_warn("Failed to apply target marker: " + str(e), throttle_s = 5.0)

        return cv2_img

    def applyCrosshairOverlay(self, cv2_img, x_ratio, y_ratio, color):
        # The click arrives as a ratio of the VIEWED image, which need not be
        # this frame's size -- the overlay is drawn on the source frame. A ratio
        # re-expressed against this frame's own dimensions lands on the same
        # point in the scene either way.
        try:
            height_px = cv2_img.shape[0]
            width_px = cv2_img.shape[1]
            x_px = int(round(float(x_ratio) * (width_px - 1)))
            y_px = int(round(float(y_ratio) * (height_px - 1)))
            if x_px < 0 or y_px < 0 or x_px >= width_px or y_px >= height_px:
                return
            size_px = max(int(max(width_px, height_px) * CROSSHAIR_SIZE_RATIO), 6)
            [_, _, line_thickness] = self.getOverlayFontDims(cv2_img)
            cv2.line(cv2_img, (x_px - size_px, y_px), (x_px + size_px, y_px), color, line_thickness)
            cv2.line(cv2_img, (x_px, y_px - size_px), (x_px, y_px + size_px), color, line_thickness)
            cv2.circle(cv2_img, (x_px, y_px), int(size_px / 2), color, thickness = line_thickness)
        except Exception as e:
            self.msg_if.pub_warn("Failed to apply crosshair overlay: " + str(e), throttle_s = 5.0)



    def shutdownCb(self):
        try:
            self.unsubscribeImgTopic()
        except Exception:
            pass


#########################################
# Main
#########################################
if __name__ == '__main__':
    AutoMoveImgPub()
