#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi engine (nepi_engine) repo
# (see https://github.com/nepi-engine/nepi_engine)
#
# License: NEPI Engine repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License", 
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
#os.environ['EGL_PLATFORM'] = 'surfaceless'   # Ubuntu 20.04+
import copy

import numpy as np
try:
    import cupy as npa
except:
    import numpy as npa
    
import cv2
import math
import random

from colormath.color_objects import LabColor, sRGBColor
from colormath.color_conversions import convert_color
from colormath.color_diff import delta_e_cie2000
import random



from scipy.spatial import distance
from scipy.ndimage.filters import convolve
from scipy.sparse import diags, csr_matrix
from scipy.sparse.linalg import spsolve


import matplotlib.pyplot as plt
import io

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from nepi_sdk.nepi_sdk import logger as Logger
logger = Logger( )


STANDARD_IMAGE_SIZES = ['630 x 900','720 x 1080','955 x 600','1080 x 1440','1024 x 768 ','1980 x 2520','2048 x 1536','2580 x 2048','3648 x 2736']


# Label font, hoisted out of the draw loop. Nothing about it varies per frame.
OVERLAY_FONT = cv2.FONT_HERSHEY_SIMPLEX
OVERLAY_FONT_COLOR = (255, 255, 255)
OVERLAY_LINE_TYPE = cv2.LINE_AA
OVERLAY_LINE_COLOR = (255, 255, 255)

ROTATE_DICT = {
'0': '0',
'90': cv2.ROTATE_90_CLOCKWISE,
'180': cv2.ROTATE_180,
'270': cv2.ROTATE_90_COUNTERCLOCKWISE,
'360': '0'
}

def get_image_publisher_namespaces(name):
    msg_type = 'sensor_msgs/Image'
    return nepi_sdk.find_topics_by_msg(msg_type)


#########################
### Misc Functions

def create_cv2_blank_img(width=900, height=630, color_rgb = (255, 255, 255) ):
  # Create a black image (all zeros)
  img = np.zeros((height, width, 3), dtype=np.uint8)

  # Draw a white circle in the center
  center_coordinates = (width // 2, height // 2)
  radius = 100
    # White color in BGR
  thickness = -1  # Fill the circle
  color_bgr = (color_rgb[2],color_rgb[1],color_rgb[0])
  cv2.circle(img, center_coordinates, radius, color_bgr, thickness)

  # Draw a blue rectangle
  start_point = (width // 4, height // 4)
  end_point = (3 * width // 4, 3 * height // 4)
  color = (255, 0, 0)  # Blue color in BGR
  thickness = 5
  img_out = copy.deepcopy(img)
  cv2.rectangle(img_out, start_point, end_point, color, -1)
  return img_out


def getImgShortName(det_namespace):
    base_namespace = nepi_sdk.get_base_namespace()
    short_name = det_namespace.replace(base_namespace,"")
    if short_name.find("idx") != -1:
        short_name = short_name.replace("/idx","")
    return short_name


def get_img_depth_map_topic(img_topic):
  topic = img_topic.rsplit('/',1)[0] + "/depth_map"
  topic = nepi_sdk.find_topic(topic)
  if topic == "":
    topic = None
  return topic
      

def get_img_pointcloud_topic(img_topic):
  topic = img_topic.rsplit('/',1)[0] + "/pointcloud"
  topic = nepi_sdk.find_topic(topic)
  if topic == "":
    topic = None
  return topic


def create_bgr_random_color(seed_value = random.random() ):
    color_bgr = (0, 0, 0)

    integer_seed = int(seed_value * 1000000) 
    random.seed(integer_seed)
    color_bgr[0] = random.randint(0, 255) 

    
    seed_value = color_bgr[0] / 255
    integer_seed = int(seed_value * 1000000) 
    random.seed(integer_seed)
    color_bgr[1] = random.randint(0, 255)

    seed_value = color_bgr[1] / 255
    integer_seed = int(seed_value * 1000000) 
    random.seed(integer_seed)
    color_bgr[2] = random.randint(0, 255)

    color_bgr = fix_color(color_bgr)

    return color_bgr


def convert_rgb2bgr(color_rgb):
   color_bgr = fix_color((color_rgb[2],color_rgb[1],color_rgb[0]))
   return color_bgr

def convert_bgr2rgb(color_bgr):
   color_rgb = fix_color((color_bgr[2],color_bgr[1],color_bgr[0]))
   return color_rgb


def adjust_bgr_contrast_color(base_bgr, contrast_bgr , min_delta_e=50):
    base_rgb = convert_bgr2rgb(base_bgr)
    base_lab = convert_color(base_rgb, LabColor)
    contrast_rgb = convert_bgr2rgb(contrast_bgr)
    delta_e = delta_e_cie2000(base_lab, contrast_rgb)
    if delta_e < min_delta_e:
        seed_value = sum(contrast_bgr)/(255*3)
        contrast_bgr = create_bgr_contrast_color(base_bgr, seed_value = seed_value)
    return contrast_bgr


def create_bgr_contrast_color(base_bgr, seed_value = random.random() , min_delta_e=50):
    base_rgb = convert_bgr2rgb(base_bgr)
    base_lab = convert_color(base_rgb, LabColor)

    attempts = 0
    max_attempts_per_color = 1000  # Prevent infinite loops
    contrast_bgr = base_bgr
    while contrast_bgr is None and attempts < max_attempts_per_color:
            # Generate a random RGB color fron seed
            random_bgr = create_bgr_random_color((seed_value))
            random_rgb = convert_bgr2rgb(random_bgr)
            random_lab = convert_color(random_rgb, LabColor)
            contrast_rgb = random_rgb.get_rgb_hex()
            # Calculate the color difference (Delta E)
            delta_e = delta_e_cie2000(base_lab, random_lab)
            if delta_e >= min_delta_e:
                contrast_bgr = convert_rgb2bgr(contrast_rgb)
              
            attempts += 1
            # Update Seed Value
            integer_seed = int(seed_value * 1000000) 
            seed_value = random.seed(integer_seed)

    if contrast_bgr is None:
       contrast_bgr = convert_rgb2bgr(contrast_rgb)
    print(contrast_bgr)
    return contrast_bgr

# # Example usage:
# base_color = "#FF0000"  # Red
# num_contrasting = 3
# contrasting_list = get_contrasting_colors(base_color, num_contrasting)
# print(f"Contrasting colors for {base_color}: {contrasting_list}")

# base_color_2 = "#008000" # Green
# num_contrasting_2 = 5
# contrasting_list_2 = get_contrasting_colors(base_color_2, num_contrasting_2, min_delta_e=60)
# print(f"Contrasting colors for {base_color_2}: {contrasting_list_2}")


def create_bgr_contrast_colormap_list(base_bgr, num_colors=256):
  colors_list = []
  for i in range(num_colors):
    color = create_bgr_contrast_color(base_bgr, i/num_colors)
    colors_list.append(color)
  return colors_list


def create_bgr_jet_color(seed_value): # seed_value -> 0-1
    seed_value = np.clip(seed_value, 0, 1)
    
    c1 = np.array([0.267004, 0.004874, 0.329415])
    c2 = np.array([0.232985, 0.298771, 0.538668])
    c3 = np.array([0.128768, 0.568284, 0.505529])
    c4 = np.array([0.529777, 0.780797, 0.307166])
    c5 = np.array([0.993248, 0.906157, 0.143936])

    if 0 <= seed_value <= 0.25:
        c_list = c1 + (c2 - c1) * (seed_value / 0.25)
    elif 0.25 < seed_value <= 0.5:
        c_list = c2 + (c3 - c2) * ((seed_value - 0.25) / 0.25)
    elif 0.5 < seed_value <= 0.75:
        c_list = c3 + (c4 - c3) * ((seed_value - 0.5) / 0.25)
    elif 0.75 < seed_value <= 1:
        c_list = c4 + (c5 - c4) * ((seed_value - 0.75) / 0.25)
    else:
        c_list = np.array([0.0, 0.0, 0.0])
    c_list = c_list * 255
    c_list = c_list.astype(int)
    return (int(c_list[2]),int(c_list[1]),int(c_list[0]))

def create_bgr_jet_colormap_list(num_colors=256):
  colors_list = []
  for i in range(num_colors):
    color = create_bgr_jet_color(i/num_colors)
    colors_list.append(color)
  return colors_list



def fix_color(color = (255,255,255)):
  for i, entry in enumerate(list(color)):
      if entry < 0:
         entry = 0
      if entry > 255:
         entry = 255
      color[i] = int(entry)
  return color
         

def get_bounding_box_image(cv2_img, bounding_box_dict):
    cv2_sub_img = None
    if cv2_img is not None:
      [height, width, channels] = cv2_img.shape
      if 'xmin' in bounding_box_dict.keys():
        try:
            xmin = bounding_box_dict['xmin']
            xmax = bounding_box_dict['xmax']
            ymin = bounding_box_dict['ymin']
            ymax = bounding_box_dict['ymax']
            success = True
        except Exception as e:
            logger.log_warn("Failed to get bounding box data: " + str(e))
            success = False
      elif 'xmin_pixel' in bounding_box_dict.keys():
        try:
            xmin = bounding_box_dict['xmin_pixel']
            xmax = bounding_box_dict['xmax_pixel']
            ymin = bounding_box_dict['ymin_pixel']
            ymax = bounding_box_dict['ymax_pixel']
            success = True
        except Exception as e:
            logger.log_warn("Failed to get bounding box data: " + str(e))
            success = False
      if success == True:
        cv2_sub_img = cv2_img[ymin:ymax, xmin:xmax]
    return cv2_sub_img

'''
def create_jet_colormap_list(num_colors=256):
    """
    Generates a jet colormap as a color tuple list.

    Args:
        num_colors: The number of colors in the colormap.

    Returns:
        A NumPy array of shape (num_colors, 3) representing the RGB colormap.
    """

    # Initialize the colormap array
    colormap_list = []
    color = [0,0,0]

    # Define the color segments
    positions = [0.0, 0.125, 0.375, 0.625, 0.875, 1.0]
    red_values = [0.0, 0.0, 1.0, 1.0, 0.0, 0.0]
    green_values = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    blue_values = [0.5, 1.0, 1.0, 0.0, 0.0, 0.5]

    # Interpolate colors for each segment
    for i in range(num_colors):
        # Normalize the index to the range [0, 1]
        x = i / (num_colors - 1)

        # Find the segment where x belongs
        for j in range(len(positions) - 1):
            if positions[j] <= x <= positions[j + 1]:
                x0 = positions[j]
                x1 = positions[j + 1]
                y_red0 = red_values[j]
                y_red1 = red_values[j + 1]
                y_green0 = green_values[j]
                y_green1 = green_values[j + 1]
                y_blue0 = blue_values[j]
                y_blue1 = blue_values[j + 1]
                break

        # Linear interpolation
        red = y_red0 + (y_red1 - y_red0) * ((x - x0) / (x1 - x0))
        green = y_green0 + (y_green1 - y_green0) * ((x - x0) / (x1 - x0))
        blue = y_blue0 + (y_blue1 - y_blue0) * ((x - x0) / (x1 - x0))

        # Assign RGB values to the colormap array
        color[0] = int(red*255)
        color[1] = int(green*255)
        color[2] = int(blue*255)

        colormap = []
        for i2 in range(3):
            colormap.append(int(color[i2])) #*255))
        colormap_list.append(colormap)

    return colormap_list
  '''

###########################################
### Image conversion functions

def rosimg_to_cv2img(ros_img_msg, encoding = 'passthrough'):
  """ Convert image from ROS to OpenCV

  Args: 
      ros_img_msg (sensor.msg.Image): ROS Image message

  Returns: 
      cv2_img (cv2.mat): OpenCV Mat Image
  """
  bridge = CvBridge()
  cv2_img = bridge.imgmsg_to_cv2(ros_img_msg, desired_encoding = encoding)
  return cv2_img
    
    
def cv2img_to_rosimg(cv2_img, encoding="bgr8"): # "bgr8", "rgb8", or "mono8"
  """ Convert image from OpenCV to ROS

  Args: 
       cv2_img (cv2.mat): OpenCV Mat Image

  Returns: 
      ros_img_msg (sensor.msg.Image): ROS Image message
  """

  bridge = CvBridge()
  ros_img_msg = bridge.cv2_to_imgmsg(cv2_img, encoding = encoding)
  return ros_img_msg


def grayscale_to_rgb(gray_image):
    """
    Converts a grayscale image to an RGB image.

    Args:
        gray_image (numpy.ndarray): A grayscale image with shape (H, W) or (H, W, 1).

    Returns:
        numpy.ndarray: An RGB image with shape (H, W, 3).
    """

    if len(gray_image.shape) == 2:
        height, width = gray_image.shape
    elif len(gray_image.shape) == 3 and gray_image.shape[2] == 1:
        height, width, _ = gray_image.shape
    else:
        raise ValueError("Input image must be grayscale with shape (H, W) or (H, W, 1)")

    rgb_image = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2RGB)
    return rgb_image
    


def npDepthMap_to_cv2ColorImg(np_depth_map, min_range_m = None, max_range_m = None, min_ratio = 0, max_ratio = 1, scaler=1e3, flip_color_map = True):

    cv2_img = None
    cv2_img_min_range_m = 0
    cv2_img_max_range_m = 0
    try:

        # Get range data
        if min_range_m is None:
            min_depth = np.nanmin(np_depth_map)
        else:
           min_depth = min_range_m * scaler

        if max_range_m is None:
            max_depth = np.nanmax(np_depth_map)
        else:
           max_depth = max_range_m * scaler

        #logger.log_warn("Depth Map has ranges: " + str([np.nanmin(np_depth_map),np.nanmax(np_depth_map)]))
        #logger.log_warn("Depth Map using ranges: " + str([min_depth,max_depth]))

        delta_depth = max_depth - min_depth
        # Adjust range based on user inputs
        max_depth_adj = min_depth + max_ratio * delta_depth
        min_depth_adj = min_depth + min_ratio * delta_depth
        delta_depth_adj = max_depth_adj - min_depth_adj

        # Filter np_depth_map in range
        np_depth_map[np.isnan(np_depth_map)] = max_depth_adj 
        np_depth_map[np.isinf(np_depth_map)] = max_depth_adj 
        np_depth_map[np_depth_map <= min_depth_adj] = max_depth_adj # set to max
        np_depth_map[np_depth_map >= max_depth_adj] = max_depth_adj # set to max


        # Create colored cv2 depth image
        np_depth_map = np_depth_map - min_depth_adj # Shift down 
        if flip_color_map == True:
           np_depth_map = np.abs(np_depth_map - max_depth_adj) # Reverse for colormap
        np_depth_map = np.array(255*np_depth_map/delta_depth_adj,np.uint8) # Scale for bgr colormap
        cv2_img = cv2.applyColorMap(np_depth_map, cv2.COLORMAP_JET)
        cv2_img_min_range_m = min_depth_adj / scaler
        cv2_img_max_range_m = max_depth_adj / scaler
    except Exception as e:
        logger.log_warn("Failed convert depth map to color img: " + str(e), throttle_s = 5.0)
    return cv2_img


def get_range_from_npDepthMap(np_depth_map,target_box,target_box_percent = 100, target_min_points = 10, target_depth_m = None):
    target_range_m = float(-999)
    if np_depth_map is not None:

        # reduce target box based on user settings
        y_len = (target_box['ymax'] - target_box['ymin'])
        x_len = (target_box['xmax'] - target_box['xmin'])
        #logger.log_warn("Got y x box lengths: " + str([y_len,x_len]))
        if target_depth_m is None:
            target_depth_m = max(x_len,y_len) / 2


        adj_ratio = float(target_box_percent )/3.5
        if target_box_percent == 100: 
            delta_y = 0
            delta_x = 0
        else:
            adj = float(target_box_percent )/3.5
            delta_y = int(y_len / 2 * (adj_ratio-1))
            delta_x = int(x_len / 2 * (adj_ratio-1))
        ymin_adj=target_box['ymin'] - delta_y
        ymax_adj=target_box['ymax'] + delta_y
        xmin_adj=target_box['xmin'] - delta_x
        xmax_adj=target_box['xmax'] + delta_x
        #logger.log_warn("Got adjusted box sizes yx: " + str([ymin_adj,ymax_adj,xmin_adj,xmax_adj]))

        # Calculate target range        
        # Get target range from cropped and filtered depth data
        depth_box_adj= np_depth_map[ymin_adj:ymax_adj,xmin_adj:xmax_adj]
        depth_array=depth_box_adj.flatten()
        depth_array = depth_array[~np.isinf(depth_array)] # remove inf entries
        depth_array = depth_array[~np.isnan(depth_array)] # remove nan entries
        depth_array = depth_array[depth_array>0] # remove zero entries
        depth_val=np.mean(depth_array) # Initialize fallback value.  maybe updated
        #logger.log_warn("got depth value " + str( depth_val))
        # Try histogram calculation
        try:
            min_range = np.min(depth_array)
            max_range = np.max(depth_array)
        except:
            min_range = 0
            max_range = 0
        delta_range = max_range - min_range
        if delta_range > target_depth_m/2:
            bins_per_target = 10
            bin_step = target_depth_m / bins_per_target
            num_bins = 1
            #logger.log_warn('delta_range: ' + str(delta_range))
            #logger.log_warn('bin_step: ' + str(bin_step))
            if bin_step > 0.001 and math.isinf(delta_range) == False :
                num_bins = int(delta_range / bin_step)
                # Get histogram
                hist, hbins = np.histogram(depth_array, bins = num_bins, range = (min_range,max_range))
                bins = hbins[1:] + (hbins[1:] - hbins[:-1]) / 2
                peak_dist = int(bins_per_target / 2)
                #max_hist_inds,ret_dict = find_peaks(hist, distance=peak_dist)
                #max_hist_inds = list(max_hist_inds)
                #max_hist_ind = max_hist_inds[0]
                max_hist_val = hist[0]
                max_hist_ind = 0
                for ih, val in enumerate(hist):
                    if val > max_hist_val:
                        max_hist_val = val
                        max_hist_ind = ih
                    elif val < max_hist_val:
                        break 
                #logger.log_warn(max_hist_ind)
                hist_len = len(hist)
                bins_len = len(bins)
                # Hanning window on targets
                win_len = bins_per_target
                if hist_len > win_len:
                    win_len_half = int(bins_per_target/2)
                    win = np.hanning(win_len)
                    han_win = np.zeros(hist_len)
                    han_win[:win_len] = win
                    win_center = int(win_len/2)
                    win_roll = max_hist_ind - win_center
                    front_pad = 0
                    back_pad = -0
                    if win_roll < 0 and max_hist_ind < win_len:
                        back_pad = win_len - max_hist_ind
                    elif win_roll > 0 and max_hist_ind > (hist_len - win_len + 1):
                        front_pad = max_hist_ind - (hist_len - win_len + 1)             
                    han_win = np.roll(han_win,win_roll)
                    han_win[:front_pad] = 0
                    han_win[back_pad:] = 0
                    han_win_len = len(han_win)
                    
                    #logger.log_warn([min_range,max_range])
                    #logger.log_warn(bins)
                    #logger.log_warn(han_win)
                    if np.sum(han_win) > .1:
                        depth_val=np.average(bins,weights = han_win)
                    


            min_filter=depth_val-target_depth_m/2
            max_filter=depth_val+target_depth_m/2
            depth_array=depth_array[depth_array > min_filter]
            depth_array=depth_array[depth_array < max_filter]
            depth_len=len(depth_array)
            #logger.log_warn("")
            #logger.log_warn("Got depth_length: " + str(depth_len))
            #logger.log_warn(depth_len)
            if depth_len > target_min_points:
                target_range_m=depth_val / 1000
            else:
                target_range_m= -999
            #logger.log_warn("Got Range in meters: " + str(target_range_m))
    return target_range_m



###########################################
### Image filter functions    

def get_bgr_filter(color_bgr, sensitivity):

  # Define a tolerance level for each R, G, B channel
  # This tolerance determines how much variation around the target color is allowed.
  tolerance = 127 - ( 127 * sensitivity ) # Example: Allow a variation of +/- 20 for each channel


  # Calculate the lower bound for the RGB filter
  # Ensure values do not go below 0
  lower_bound_bgr = (
      int(max(0, color_bgr[0] - tolerance)),
      int(max(0, color_bgr[1] - tolerance)),
      int(max(0, color_bgr[2] - tolerance))
    )

  # Calculate the upper bound for the RGB filter
  # Ensure values do not exceed 255
  upper_bound_bgr = (
      int(min(255, color_bgr[0] + tolerance)),
      int(min(255, color_bgr[1] + tolerance)),
      int(min(255, color_bgr[2] + tolerance))   
    )

  return [lower_bound_bgr,upper_bound_bgr]

    
###########################################
### Image manipulation functions

def get_aspect_ratios(ratio, max_limit: int = 10) -> tuple:
    """
    Given an aspect ratio as a float (width / height), returns a tuple 
    of the closest integer (width, height) aspect ratios bounded by max_limit.
    """
    w_ratio, h_ratio = 1, 1

    min_diff = float('inf')
    
    # Iterate through all possible heights up to the limit
    for h in range(1, max_limit + 1):
        # Calculate the closest integer width for the current height
        w = round(ratio * h)
        if w == 0:
            w = 1
            
        current_ratio = w / h
        diff = abs(current_ratio - ratio)
        
        # Keep track of the pair with the smallest error
        if diff < min_diff:
            min_diff = diff
            w_ratio, h_ratio = w, h

    return w_ratio, h_ratio

def get_aspect_ratio_clean(ratio):
    [w_ratio, h_ratio] = get_aspect_ratios(ratio)
    aspect_ratio = round(w_ratio / h_ratio, 3)
    return aspect_ratio


def get_aspect_ratio_str(ratio):
    ratio = get_aspect_ratio_clean(ratio)
    [w_ratio, h_ratio] = get_aspect_ratios(ratio)
    aspect_ratio_str = str(w_ratio) + ":" + str(h_ratio)
    return aspect_ratio_str


            
def adjust_aspect_ratio(cv2_img, target_ratio):
    cropped = cv2_img
    if target_ratio >= 0.5 and target_ratio <= 2.5:
        # Get current dimensions
        h, w = cv2_img.shape[:2]
        current_ratio = w / h
        
        if current_ratio > target_ratio:
            # Cv2_Img is too wide: crop the sides
            new_w = int(h * target_ratio)
            start_x = (w - new_w) // 2
            cropped = cv2_img[0:h, start_x:start_x + new_w]
        else:
            # Cv2_Img is too tall: crop the top and bottom
            new_h = int(w / target_ratio)
            start_y = (h - new_h) // 2
            cropped = cv2_img[start_y:start_y + new_h, 0:w]
        
    return cropped

def resize_proportionally(cv2_img, max_width, max_height, interp = cv2.INTER_NEAREST):
    height, width = cv2_img.shape[:2]

    if max_width is None and max_height is None:
        return cv2_img
    
    if max_width is None:
        ratio = max_height / height
        max_width = int(width * ratio)
    elif max_height is None:
        ratio = max_width / width
        max_height = int(height * ratio)
    else:
        ratio = min(max_width / width, max_height / height)
        new_width = int(width * ratio)
        new_height = int(height * ratio)

    resized_cv2_img = cv2.resize(cv2_img, (new_width, new_height), interpolation=interp)
    return resized_cv2_img, ratio, new_width, new_height


def rotate_degrees(cv2_img, deg=0):
    # Normalize any integer angle to the range 0-359 (360/multiples collapse to 0).
    #deg = int(deg) % 360
    deg_str=str(deg)
    if deg == 0:
        return cv2_img
    elif deg_str in ROTATE_DICT.keys():
        if ROTATE_DICT[deg_str] != '0':
            cv2_img = cv2.rotate(cv2_img, ROTATE_DICT[deg_str])   
    elif abs(deg) > 0.01:     
        # Rotate about the image center while keeping the OUTPUT size identical to the
        # input (same width and height) so the display box never changes dimensions or
        # aspect ratio. Uncovered corners are filled with solid black (borderValue=0,
        # BORDER_CONSTANT). getRotationMatrix2D treats a POSITIVE angle as
        # counter-clockwise, so we negate to keep the original clockwise direction
        # (matching the old cv2.ROTATE_90_CLOCKWISE behavior for 90 deg; 180 and
        # 270 likewise render as before).
        #logger.log_warn("Live Adjust Rotating degrees: " + str(deg))
        height, width = cv2_img.shape[:2]
        center = (width / 2.0, height / 2.0)
        rot_mat = cv2.getRotationMatrix2D(center, -deg, 1.0)
        cv2_img = cv2.warpAffine(cv2_img, rot_mat, (width, height),
                                flags=cv2.INTER_LINEAR,
                                borderMode=cv2.BORDER_CONSTANT,
                                borderValue=0)
    return cv2_img

def translate_pixels(cv2_img, shift_x, shift_y):
    """
    Translates an OpenCV image by a given number of pixels in x and y directions.
    
    Positive shift_x: moves image right
    Negative shift_x: moves image left
    Positive shift_y: moves image down
    Negative shift_y: moves image up
    """
    # Get image dimensions (height, width)
    height, width = cv2_img.shape[:2]

    if abs(shift_x) > width:
       shift_x = np.sign(shift_x) * width
    if abs(shift_y) > height:
       shift_y = np.sign(shift_y) * height
    
    # Create the 2x3 translation matrix
    # [1, 0, shift_x]
    # [0, 1, shift_y]
    translation_matrix = np.float32([[1, 0, shift_x], [0, 1, shift_y]])
    
    # Apply the affine transformation
    translated_cv2_img = cv2.warpAffine(cv2_img, translation_matrix, (width, height))
    
    return translated_cv2_img

def flip_horz(cv2_img):
    cv2_img = cv2.flip(cv2_img, 1)          
    return cv2_img

def flip_vert(cv2_img):
    cv2_img = cv2.flip(cv2_img, 0)         
    return cv2_img
  
      

###########################################
### Image process functions
def is_gray(cv2_img):
    cv_shape = cv2_img.shape
    if len(cv_shape) == 2:
      return True
    elif len(cv_shape) == 3 and cv_shape[2] == 1:
      return True
    else:
      return False

def adjust_auto(cv2_img, sensitivity_ratio = 0.0):    
  # Apply Image Enhancment
  cv2_img.setflags(write=1)
  # Apply threshold filter
  cv2_img=adjust_sharpness(cv2_img, sensitivity_ratio = 0.2)
  #logger.log_info("input image shape and type")
  #logger.log_info(cv2_img.shape)
  if is_gray(cv2_img) is True:
    gray = cv2_img
  else:
    # Color Correction optimization
    Max=[0,0,0]
    for k in range(0, 3):
      Max[k] = np.max(cv2_img[:,:,k])
    Min_Max_channel  = np.min(Max)
    for k in range(0, 3):
      Max_channel  = np.max(cv2_img[:,:,k])
      Min_channel  = np.min(cv2_img[:,:,k])
      Mean_channel = np.mean(cv2_img[:,:,k])
      Chan_scale = (255 - Mean_channel) / (Max_channel - Min_channel)
      if Chan_scale < 1:
        Chan_scale = 1 - (1-Chan_scale)*(255-Min_Max_channel)/170
      elif Chan_scale > 1:
        Chan_scale = 1 + (Chan_scale-1)*(255-Min_Max_channel)/170
      if Chan_scale > 1*(1+sensitivity_ratio):
        Chan_scale = 1 *(1+sensitivity_ratio)
      if Chan_scale < -1*(1+sensitivity_ratio):
        Chan_scale = -1 *(1+sensitivity_ratio)
      Chan_offset = -1*Min_channel
      if Chan_offset < -10 * (1+9*sensitivity_ratio):
        Chan_offset = -10 * (1+9*sensitivity_ratio)
    cv2_img[:,:,k] = (cv2_img[:,:,k] + Chan_offset) * Chan_scale   
    # Contrast and Brightness optimization
    gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
  #logger.log_info("gray image shape and type")
  #logger.log_info(gray.shape)
  # Calculate grayscale histogram
  hist = cv2.calcHist([gray],[0],None,[256],[0,256])
  hist_size = len(hist)
  # Calculate cumulative distribution from the histogram
  accumulator = []
  accumulator.append(float(hist[0]))
  for index in range(1, hist_size):
    accumulator.append(accumulator[index -1] + float(hist[index]))
  # Locate points to clip
  maximum = accumulator[-1]
  clip_hist_percent = (maximum/100)
  clip_hist_percent /= 2.0
  # Locate left cut
  minimum_gray = 0
  while accumulator[minimum_gray] < clip_hist_percent:
    minimum_gray += 5
  # Locate right cut
  maximum_gray = hist_size -1
  while accumulator[maximum_gray] >= (maximum - clip_hist_percent):
    maximum_gray -= 1
  # Calculate alpha and beta values
  alpha = 255 / (maximum_gray - minimum_gray) * (sensitivity_ratio*4)
  if alpha>2: ##
    alpha=2 ##
  beta = (-minimum_gray * alpha + 10) * (sensitivity_ratio*4)
  if beta<-50: ##
    beta=-50 ##
  cv2_img = cv2.convertScaleAbs(cv2_img, alpha=alpha, beta=beta)
  return cv2_img
    
def adjust_contrast(cv2_img, sensitivity_ratio = 0.5):
  if sensitivity_ratio < 0.49 or sensitivity_ratio > 0.51:
    max_value = 200
    contrast = int((sensitivity_ratio - 0.5) * 2 * max_value) # +- max_value
    if abs(contrast) > max_value:
      contrast = np.sign(contrast) * max_value
    if contrast != 0:
      if contrast > 0:
        shadow = contrast
        highlight = 255
      else:
        shadow = 0
        highlight = 255 + contrast
    alpha = (highlight - shadow)/255
    gamma = shadow
    cv2_img = cv2.addWeighted(cv2_img, alpha, cv2_img, 0, gamma)
  return cv2_img

def adjust_brightness(cv2_img, sensitivity_ratio = 0.5):
  if sensitivity_ratio < 0.49 or sensitivity_ratio > 0.51:
    max_value = 90
    brightness = int((sensitivity_ratio - 0.5) * 2 * max_value) # +- max_value
    if abs(brightness) > max_value:
      brightness = np.sign(brightness) * max_value
    f = 131*(brightness + 127)/(127*(131-brightness))
    alpha = f
    gamma = 127*(1-f)
    
    #cv2.convertScaleAbs(cv2_img, cv2_img, alpha_c, gamma_c)
    cv2_img = cv2.addWeighted(cv2_img, alpha, cv2_img, 0, gamma)
  return cv2_img

def adjust_sharpness(cv2_img, sensitivity_ratio = 0.0):
  if sensitivity_ratio > 0.05:
    # gaussian kernel for sharpening
    gaussian_blur = cv2.GaussianBlur(cv2_img,(7,7),sigmaX=2)
    # sharpening using addWeighted()
    sharpness_min = 2
    sharpness_max = 10
    #sharpness = int(sharpness_min + sensitivity_ratio * (sharpness_max-sharpness_min))
    sharpness = int(sharpness_min + sensitivity_ratio**2 * (sharpness_max-sharpness_min))
    cv2_img = cv2.addWeighted(cv2_img,sharpness,gaussian_blur,-(sharpness-1),0)
    #cv2_img * sensitivity_ratio
  return cv2_img

def adjust_resolution_ratio(cv2_img, ratio):
  if ratio < 0.1:
    ratio = 0.1
  if ratio > .99:
    ratio = 1
  if ratio != 1:
    new_resolution = (int(cv2_img.shape[1] * ratio),int(cv2_img.shape[0] * ratio))
    cv2_img = cv2.resize(cv2_img,(new_resolution), 0, 0, interpolation = cv2.INTER_NEAREST)
  return cv2_img,cv2_img.shape

def adjust_framerate_ratio(current_fps,ratio):
  adj_fr = 7
  if current_fps is not None:
    if ratio < 0.1:
        ratio = 0.1
    if ratio > .99:
        ratio = 1
    if ratio != 1:
        adj_fr = ratio * current_fps
    else:
        adj_fr = current_fps
  return adj_fr

def get_contours(cv2_img):
  """ Calculate image contours

  Args: 
       cv2_img (cv2.mat): OpenCV Mat Image

  Returns: 
      contrours3 : OpenCV contours list
      hierarchy3 : 
  """
  #cv2_img.setflags(write=1)
  cv2_mat_gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
  ret, thresh2 = cv2.threshold(cv2_mat_gray, 150, 255, cv2.THRESH_BINARY)
  contours3, hierarchy3 = cv2.findContours(thresh2, cv2.RETR_LIST, 
                                       cv2.CHAIN_APPROX_NONE)
  return contours3, hierarchy3

def denoise_filter(cv2_img, filter_type='gaussian', kernel_size=5):
    """
    Applies a specified noise filter to an image using OpenCV.

    Args:
        image_path (str): Path to the input image.
        filter_type (str): Type of filter to apply ('gaussian', 'median', 'bilateral').
        kernel_size (int): Size of the kernel for Gaussian and Median filters.
                           For bilateral, it's the diameter of the pixel neighborhood.

    Returns:
        numpy.ndarray: The denoised image.
    """

    if filter_type == 'gaussian':
        denoised_cv2_img = cv2.GaussianBlur(cv2_img, (kernel_size, kernel_size), 0)
    elif filter_type == 'median':
        denoised_cv2_img = cv2.medianBlur(cv2_img, kernel_size)
    elif filter_type == 'bilateral':
        # Bilateral filter requires more parameters:
        # d: Diameter of pixel neighborhood
        # sigmaColor: Filter sigma in the color space
        # sigmaSpace: Filter sigma in the coordinate space
        denoised_cv2_img = cv2.bilateralFilter(cv2_img, kernel_size, 75, 75)
    else:
        logger.log_warn("Invalid filter type. Choose 'gaussian', 'median', or 'bilateral'.")
        return None

    return denoised_cv2_img


###########################################
### Image overlay functions

def overlay_contours(cv2_img,contours3, color_rgb = (0, 255, 0)):
    cv2_img_out = copy.deepcopy(cv2_img)
    cv2.drawContours(cv2_img_out, contours3, -1, color_rgb, 2, OVERLAY_FONT)
    return cv2_img_out




def get_optimal_font_dims(cv2_img, font_scale = 2e-3, thickness_scale = 1.5e-3, scale_ratio = 1):
    shape = cv2_img.shape
    h=shape[0]
    w=shape[1]
    font_scale = min(w, h) * font_scale * scale_ratio
    thickness = math.ceil(min(w, h) * thickness_scale * scale_ratio)
    return font_scale, thickness  

def get_optimal_text_size(cv2_img, text):

    cv2_shape = cv2_img.shape
    img_width = cv2_shape[1] 
    img_height = cv2_shape[0] 

    font = OVERLAY_FONT
    scale = 1.5e-3 - 0.1e-3 * math.ceil(max([img_height, img_width])/700)
    fontScale, fontThickness  = get_optimal_font_dims(cv2_img,font_scale = scale, thickness_scale = scale, scale_ratio = 0.5) 

    text_size = cv2.getTextSize(text, 
                            font, 
                            fontScale,
                            fontThickness)
    return text_size


def overlay_text(cv2_img, text, x_px = 10 , y_px = 10, color_rgb = (0, 255, 0), scale = None, thickness = None, background_rgb = None, apply_shadow = True):
    # Add text overlay
    if scale is None or thickness is None:
        new_scale, thickness  = get_optimal_font_dims(cv2_img,font_scale = 2e-3, thickness_scale = 1.5e-3)
        if scale is None:
           scale = new_scale
        else:
           thickness = thickness * (2 * scale)

    
    font = OVERLAY_FONT
    fontColor              = (color_rgb[2],color_rgb[1],color_rgb[0])
    lineType               = 1
    bottomLeftCornerOfText = (x_px,y_px)

    fontColorBlk = (0,0,0)

    # Add Background Box box if requested
    if background_rgb is not None:
      text_size = cv2.getTextSize(text, 
          font, 
          scale,
          thickness)    
      line_height = text_size[0][1]
      line_width = text_size[0][0]
      x_padding = int(line_height*0.4)
      y_padding = int(line_height*0.4)
      # Create Text Background Box

      bot_left_box =  (x_px - x_padding , y_px + y_padding)
      top_right_box = (x_px + line_width + x_padding, y_px - line_height - y_padding )

      try:
          cv2.rectangle(cv2_img, bot_left_box, top_right_box, background_rgb , -1)
      except Exception as e:
           logger.log_warn("Failed to add text background box: " + str(e), throttle_s = 5)

    # Add Text Shadow if requested
    if apply_shadow == True:
      try:
          cv2.putText(cv2_img,text, 
              bottomLeftCornerOfText, 
              font, 
              scale,
              fontColorBlk,
              thickness*2,
              lineType)
      except Exception as e:
           logger.log_warn("Failed to apply text shadow: " + str(e), throttle_s = 5)

  
    # Overlay Text
    try:
        cv2.putText(cv2_img,text, 
            bottomLeftCornerOfText, 
            font, 
            scale,
            fontColor,
            thickness,
            lineType)
    except Exception as e:
         logger.log_warn("Failed to apply overlay text: " + str(e), throttle_s = 5)
    return cv2_img
 
def overlay_text_list(cv2_img, text_list, x_px = 10 , y_px = 10, color_rgb = (0, 255, 0),
    scale = None, thickness = None, size_ratio = 0.5, background_rgb = None, apply_shadow = True):
    # Add text overlay
    x_px = int(x_px)
    y_px = int(y_px)
    font = OVERLAY_FONT
    lineType = 1
    if scale is None or thickness is None:
        scale, thickness  = get_optimal_font_dims(cv2_img,font_scale = 2e-3, thickness_scale = 1.5e-3)
    scale = scale * (0.5 + size_ratio)
    for i in range(len(text_list)):
      text = text_list[i]
      text_size = cv2.getTextSize(text, 
          font, 
          scale,
          thickness)
      # logger.log_warn("Text Size: " + str(text_size))
      line_height = text_size[0][1]
      line_width = text_size[0][0]
      y_px = int(y_px + line_height * 1.5)
      #logger.log_warn("overlaying text: " + str(text) + ' at: ' + str([x_px,y_px]))
      cv2_img = overlay_text(cv2_img, text, x_px , y_px, color_rgb, scale, thickness, background_rgb = background_rgb, apply_shadow = apply_shadow)

    return cv2_img

    
def overlay_rectangle(cv2_img,bot_left_px, top_right_px, color=(255,0,0), alpha = 0.4):
      overlay = cv2_img.copy()
      cv2.rectangle(overlay, bot_left_px, top_right_px, color, -1)
      cv2_img = cv2.addWeighted(overlay, alpha, cv2_img, 1 - alpha, 0)
      return cv2_img

def overlay_crosshair(cv2_img, x_px, y_px, color_rgb=(0, 255, 0), size_ratio = 0.5, thickness_ratio = 0.5, size=None, thickness=None,  text_list = [], text_ratio = 0.5):
    """Draws a crosshair on an image at a given (x_px, y_px) position."""
    # Draw vertical line
    # if size is None or thickness is None:
    #     size, thickness  = get_optimal_font_dims(cv2_img,font_scale = 2e-3, thickness_scale = 1.5e-3)
    shape = cv2_img.shape
    h=shape[0]
    w=shape[1]  
    hw_min = min(h,w)
    hw_size = math.ceil((0.05 * hw_min))  
    hw_thickness = math.ceil((0.004 * hw_min))
    if size is None:
       size = hw_size

    if thickness is None:
       thickness = hw_thickness
   
    size = int(size / 3 + size * (size_ratio * 1.5 ) )
    thickness = int(thickness * (thickness_ratio * 2))
    if size < 5:
       size = 5
    if thickness < 1:
       thickness = 1 
    size_ratio = nepi_utils.check_ratio(size_ratio)
    color_bgr = (color_rgb[2],color_rgb[1],color_rgb[0])
    cv2.line(cv2_img, (x_px, y_px - size), (x_px, y_px + size), color_bgr, thickness)
    # Draw horizontal line
    cv2.line(cv2_img, (x_px - size, y_px), (x_px + size, y_px), color_bgr, thickness)
    if len(text_list) > 0:
        x_px = x_px
        y_padding = math.ceil((0.02 * h) + size / 2 ) 
        y_px = y_px + y_padding
        cv2_img  =  overlay_text_list(cv2_img, text_list, x_px = x_px , y_px = y_px, color_rgb = color_rgb, size_ratio = text_ratio / 3)
    return cv2_img


def overlay_target(cv2_img, x_px, y_px, color_rgb=(0, 255, 0), size_ratio = 0.5, thickness_ratio = 0.5, size=None, thickness=None,  text_list = [], text_ratio = 0.5):
    """Draws a crosshair on an image at a given (x_px, y_px) position."""
    # Draw vertical line
    # if size is None or thickness is None:
    #     size, thickness  = get_optimal_font_dims(cv2_img,font_scale = 2e-3, thickness_scale = 1.5e-3)
    shape = cv2_img.shape
    h=shape[0]
    w=shape[1]  
    hw_min = min(h,w)
    hw_size = math.ceil((0.05 * hw_min))  
    hw_thickness = math.ceil((0.004 * hw_min))
    if size is None:
       size = hw_size

    if thickness is None:
       thickness = hw_thickness
    size_ratio = nepi_utils.check_ratio(size_ratio)
    size = int(size / 3 + size * (size_ratio * 1.5) )
    thickness = int(thickness * (thickness_ratio * 2))
    if size < 5:
       size = 5
    if thickness < 1:
       thickness = 1 
    
    # Draw vetical lines
    color_bgr = (color_rgb[2],color_rgb[1],color_rgb[0])
    cv2.line(cv2_img, (x_px, y_px + size), (x_px, y_px + 2 * size), color_bgr, thickness)
    cv2.line(cv2_img, (x_px, y_px - size), (x_px, y_px - 2 * size), color_bgr, thickness)
    # Draw horizontal lines
    cv2.line(cv2_img, (x_px + size, y_px), (x_px + 2 * size, y_px), color_bgr, thickness)
    cv2.line(cv2_img, (x_px - size, y_px), (x_px - 2 * size, y_px), color_bgr, thickness)
    # Draw circle
    center_coordinates = (x_px, y_px)
    radius = int(size + size/2)
    cv2.circle(cv2_img, center_coordinates, radius, color_bgr, thickness)

    if len(text_list) > 0:
        x_px = x_px
        y_padding = 0 # math.ceil((0.02 * h) + size ) 
        y_px = y_px + y_padding
        x_padding = math.ceil((0.02 * w) + size ) 
        x_px = x_px + x_padding
        cv2_img  =  overlay_text_list(cv2_img, text_list, x_px = x_px , y_px = y_px, color_rgb = color_rgb, size_ratio = text_ratio / 3)
    return cv2_img

def overlay_box(cv2_img, color_rgb = (255,255,255), x_px = 10, y_px = 10, w_px = 20, h_px = 20):
    # Add status box overlay
    cv2_img_out = copy.deepcopy(cv2_img)
    cv2.rectangle(cv2_img_out, (x_px, y_px), (w_px, h_px), color_rgb, -1)

def overlay_bounding_box(cv2_img,bot_left_px, top_right_px, line_color=(255,0,0), line_thickness=2):
    try:
        cv2.rectangle(cv2_img, bot_left_px, top_right_px, line_color, thickness=line_thickness)
        success = True
    except Exception as e:
         logger.log_warn("Failed to create bounding box rectangle: " + str(e), throttle_s = 5)
    return cv2_img





def overlay_np_depth_map(cv2_img, np_depth_map,
                        min_range_m = 0,
                        max_range_m = 100,
                        ):
    
    if np_depth_map is None:
        return cv2_img

    # Publish and Save org Image Data if Required  
    [height,width] = np_depth_map.shape[0:2]

    cv2_dm_img = None
    try:
        cv2_dm_img = npDepthMap_to_cv2ColorImg(np_depth_map, min_range_m = min_range_m, max_range_m = max_range_m)      
    except Exception as e:
        pass

    if cv2_dm_img is not None:
        try:
            mask = np.any(cv2_dm_img > 0, axis=2)
            cv2_img[mask] = cv2_dm_img[mask]
        except:
            pass

    return cv2_img



      
def create_blank_image(image_size = (350, 700, 3) ):
    # Create a blank img for when not running
    cv2_img = np.zeros(image_size, dtype = np.uint8) # Empty Black Image
    return cv2_img


def create_message_image(message, image_size = (350, 700, 3),color_rgb = (0, 255, 0) ):
    # Create a blank img for when not running
    cv2_img = create_blank_image(image_size) # Empty Black Image
    # Overlay text data on OpenCV image

    scale, thickness  = get_optimal_font_dims(cv2_img,font_scale = 2e-3, thickness_scale = 1.5e-3)

    font = cv2.FONT_HERSHEY_DUPLEX
    fontColor              = color_rgb
    lineType               = 1
    text2overlay=message
    bottomLeftCornerOfText = (50,50)
    cv2.putText(cv2_img,text2overlay, 
        bottomLeftCornerOfText, 
        font, 
        scale,
        fontColor,
        thickness,
        lineType)
    return cv2_img


def rgb_to_hex(rgb_tuple):
  """
  Converts an RGB tuple (0-255) to a hexadecimal color string.
  e.g., (255, 0, 0) -> '#FF0000'
  """
  return "#{:02X}{:02X}{:02X}".format(*rgb_tuple)
    
###########################################
### Image saving functions

def read_image_file(file_path):
    cv2_img = None
    if os.path.exists(file_path):
        try:
            cv2_img = cv2.imread(file_path)
            if cv2_img is not None:
                success = True
        except Exception as e:
            logger.log_warn("Failed to get cv2_img from file: " + file_path + " " + str(e), throttle_s = 5.0)
    else:
        logger.log_warn("Failed to find image file: " + file_path, throttle_s = 5.0)
    return cv2_img

def write_image_file(file_path,cv2_img):
    success = False
    path = os.path.dirname(file_path)
    if os.path.exists(path):
        try:
            success = cv2.imwrite(file_path, cv2_img)
        except Exception as e:
            logger.log_warn("Failed to write image to file: " + file_path + " " + str(e), throttle_s = 5.0)
    else:
        logger.log_warn("Failed to find file path: " + path, throttle_s = 5.0)
    return success



#########################
# Image Analysis Functions


def adjust_bgr_color(color_bgr = (147, 175, 35), h_sensitivity = 0.5, s_sensitivity = 0.5, v_sensitivity = 0.5):

    color_array = np.array([[[color_bgr[0], color_bgr[1], color_bgr[2]]]], dtype=np.uint8)
    hsv_color = cv2.cvtColor(color_array, cv2.COLOR_BGR2HSV)[0,0,:]
    #logger.log_warn("Using HSV Color " + str(hsv_color ))

  
  
    hsv_color_adjusted = np.uint8([[[
      int(min(90, max(0, hsv_color[0] + 90 * (-0.5 + h_sensitivity )))),
      int(min(255, max(0, hsv_color[1] + 128 * (-0.5 + s_sensitivity )))),
      int(min(255, max(0, hsv_color[2] + 128 * (-0.5 + v_sensitivity ))))
    ]]])
    logger.log_warn("Got hbr adjust color: " + str(hsv_color_adjusted))
    # hsv_color = np.uint8([[[120, 255, 255]]]) # Example: Pure Green in OpenCV HSV
    bgr_color = cv2.cvtColor(hsv_color_adjusted, cv2.COLOR_HSV2BGR)
    logger.log_warn("Got bgr adjust color: " + str(bgr_color))

    color_bgr_adjusted = (int(bgr_color[0,0,0]),int(bgr_color[0,0,1]),int(bgr_color[0,0,2]))
    return color_bgr_adjusted



def create_color_mask(cv2_img, color_bgr = (147, 175, 35), sensitivity = 0.5 , hscalers = [2,2], sscalers = [2,2], vscalers = [2,2]):

    # Convert to HSV and isolate laser color (example for red laser)
    #logger.log_warn("Calc Contours for image size: " + str(cv2_img.shape))
    hsv = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)

    # Define range filters for color in HSV
    #logger.log_warn("Using BGR Color " + str(color_bgr ))

    color_array = np.array([[[color_bgr[0], color_bgr[1], color_bgr[2]]]], dtype=np.uint8)
    hsv_color = cv2.cvtColor(color_array, cv2.COLOR_BGR2HSV)[0,0,:]
    #logger.log_warn("Using HSV Color " + str(hsv_color ))

    sensitivity = 1 - (sensitivity * 0.5 + 0.1)

    
  
    lower_bound = np.array([
      int(max(0, hsv_color[0] - 90 * sensitivity / hscalers[0] )),
      int(max(0, hsv_color[1] - 128 * sensitivity / sscalers[0] )),
      int(max(0, hsv_color[2] - 128 * sensitivity / vscalers[0]))
    ])


    upper_bound = np.array([
      int(min(90, hsv_color[0] + 90 * sensitivity / hscalers[1] )),
      int(min(255, hsv_color[1] + 128 * sensitivity / sscalers[1])),
      int(min(255, hsv_color[2] + 128 * sensitivity / vscalers[1]))
      
    ])

    #logger.log_warn("Using Mask Colors " + str([lower_bound, upper_bound]))

    # Threshold the HSV image to get only red colors
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    return mask


def pad_to_ratio(cv2_img, ratio , color_bgr = (0,0,0)):
    cv2_shape = cv2_img.shape
    img_width = cv2_shape[1] 
    img_height = cv2_shape[0] 
    cratio = img_width/img_height
    if ratio != 1 and ratio != cratio and ratio > 0.01:
        [top, bottom, left, right] = [0,0,0,0]
        
        if cratio > ratio:
            pad = int((img_width/ratio - img_height)/2)
            [top, bottom] = [pad,pad]
        elif cratio < ratio:
            pad = int((img_height * ratio - img_width)/2)
            [left, right] = [pad,pad]
           
        cv2_img = cv2.copyMakeBorder(cv2_img, top, bottom, left, right, cv2.BORDER_CONSTANT, color_bgr)
    return cv2_img
    '''
  def getShapeDistribution(cv2_img,x_px = None, y_px = None, w_px = None, h_px = None):
    https://www.geeksforgeeks.org/how-to-detect-shapes-in-images-in-python-using-opencv/
    # converting image into grayscale image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # setting threshold of gray image
    _, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    # using a findContours() function
    contours, _ = cv2.findContours(
        threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    i = 0

    # list for storing names of shapes
    for contour in contours:

        # here we are ignoring first counter because 
        # findcontour function detects whole image as shape
        if i == 0:
            i = 1
            continue

        # cv2.approxPloyDP() function to approximate the shape
        approx = cv2.approxPolyDP(
            contour, 0.01 * cv2.arcLength(contour, True), True)
        
        # using drawContours() function
        cv2.drawContours(img, [contour], 0, (0, 0, 255), 5)

        # finding center point of shape
        M = cv2.moments(contour)
        if M['m00'] != 0.0:
            x = int(M['m10']/M['m00'])
            y = int(M['m01']/M['m00'])

        # putting shape name at center of each shape
        if len(approx) == 3:
            cv2.putText(img, 'Triangle', (x, y),
                        OVERLAY_FONT, 0.6, (255, 255, 255), 2)

        elif len(approx) == 4:
            cv2.putText(img, 'Quadrilateral', (x, y),
                        OVERLAY_FONT, 0.6, (255, 255, 255), 2)

        elif len(approx) == 5:
            cv2.putText(img, 'Pentagon', (x, y),
                        OVERLAY_FONT, 0.6, (255, 255, 255), 2)

        elif len(approx) == 6:
            cv2.putText(img, 'Hexagon', (x, y),
                        OVERLAY_FONT, 0.6, (255, 255, 255), 2)

        else:
            cv2.putText(img, 'circle', (x, y),
                        OVERLAY_FONT, 0.6, (255, 255, 255), 2)
        
    '''

    '''
    def getColorDistribution(cv2_img,x_px = None, y_px = None, w_px = None, h_px = None):
      https://www.kaggle.com/code/caralosal/distribution-of-color-names-in-an-image
      # We specify each color name along with its values in the RGB channels.
      colors = ['black', 'white', 'red', 'green', 'blue', 'yellow', 'magenta', 'cyan']
      rgb_color = [[0,0,0], [255,255,255],[255,0,0], [0,255,0], [0,0,255],[255,255,0],[255,0,255],[0,255,255]]

      # CV2 reads the image as BGR instead of RGB, that's why we need to convert it after
      image = cv2.imread('/kaggle/input/color-example/image_example.jpg')

      # Convert from BGR to RGB
      image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

      # Matplotlib uses the RGB channel
      plt.imshow(image)
      plt.axis('off')
      plt.show()

    def distance(x,y):
        
        """Computes the distance between two points in a 3d space"""
        
        return np.sqrt(np.sum(np.square(np.array(x) - np.array(y))))

    def get_color_name_rgb(rgb, colors = colors, rgb_color = rgb_color):
        
        """Receives a pixel in RGB format and returns the name of the color of that pixel"""
        
        # Computes the distance between the pixel and each corner of the 3d cube
        each_distance = [distance(rgb, channel) for channel in rgb_color]
        
        # We get the index of the nearest color name to that pixel
        near_color = np.argmin(each_distance)
        
        # Return the color
        return colors[near_color]

    # Example of an specific pixel (Yellow is (255, 255,0) so (220, 220,0) is yellow)
    r = 220
    g = 220
    b = 0
    print(get_color_name_rgb((r, g, b)))



    def get_color_proportions_rgb(image):
        
        """Receieves an image in RGB format and returns the color proportions of the image"""
        
        red = image[:,:,0].ravel()
        green = image[:,:,1].ravel()
        blue = image[:,:,2].ravel()
        
        list_colors = [get_color_name_rgb((r, g, b)) for r, g, b in zip(red, green, blue)]
        
        # Show the image:
        plt.imshow(image)
        plt.axis('off')
        plt.show()

        return pd.DataFrame({'Percentage': round(pd.DataFrame(list_colors).value_counts() / len(list_colors) * 100,2)})

    get_color_proportions_rgb(image)


    
    '''
    
    
################################
# Low Light Enhancment Dual Illumination
# github.com/pvnieo/Low-light-Image-Enhancement


def low_light_filter(cv2_img, ratio):
    """Enhance input image, using either DUAL method, or LIME method. For more info, please see original papers.

    Arguments:
        cv2_img {np.ndarray} -- input image to be corrected.
        gamma {float} -- gamma correction factor.
        lambda_ {float} -- coefficient to balance the terms in the optimization problem (in DUAL and LIME).

    Keyword Arguments:
        sigma {int} -- Spatial standard deviation for spatial affinity based Gaussian weights. (default: {3})
        bc {float} -- parameter for controlling the influence of Mertens's contrast measure. (default: {1})
        bs {float} -- parameter for controlling the influence of Mertens's saturation measure. (default: {1})
        be {float} -- parameter for controlling the influence of Mertens's well exposedness measure. (default: {1})

    Returns:
        np.ndarray -- image exposure enhanced. same shape as `cv2_img`.
    """

    gamma = ratio
    lambda_ = 0.3 * ratio
    sigma = int(round(3 * ratio,0))
    bc = ratio
    bs = ratio
    be = ratio


    # create spacial affinity kernel
    kernel = create_spacial_affinity_kernel(sigma)

    # correct underexposudness
    eps = 1e-3
    im_normalized = cv2_img.astype(float) / 255.
    under_corrected = correct_underexposure(im_normalized, gamma, lambda_, kernel, eps)

    dual = True
    if dual:
        # correct overexposure and merge if DUAL method is selected
        inv_im_normalized = 1 - im_normalized
        over_corrected = 1 - correct_underexposure(inv_im_normalized, gamma, lambda_, kernel, eps)
        # fuse images
        im_corrected = fuse_multi_exposure_images(im_normalized, under_corrected, over_corrected, bc, bs, be)
    else:
        im_corrected = under_corrected

    # convert to 8 bits and returns
    return np.clip(im_corrected * 255, 0, 255).astype("uint8")

def create_spacial_affinity_kernel(spatial_sigma: float, size: int = 15):
    """Create a kernel (`size` * `size` matrix) that will be used to compute the he spatial affinity based Gaussian weights.

    Arguments:
        spatial_sigma {float} -- Spatial standard deviation.

    Keyword Arguments:
        size {int} -- size of the kernel. (default: {15})

    Returns:
        np.ndarray - `size` * `size` kernel
    """
    kernel = np.zeros((size, size))
    for i in range(size):
        for j in range(size):
            kernel[i, j] = np.exp(-0.5 * (distance.euclidean((i, j), (size // 2, size // 2)) ** 2) / (spatial_sigma ** 2))

    return kernel


def compute_smoothness_weights(L: np.ndarray, x: int, kernel: np.ndarray, eps: float = 1e-3):
    """Compute the smoothness weights used in refining the illumination map optimization problem.

    Arguments:
        L {np.ndarray} -- the initial illumination map to be refined.
        x {int} -- the direction of the weights. Can either be x=1 for horizontal or x=0 for vertical.
        kernel {np.ndarray} -- spatial affinity matrix

    Keyword Arguments:
        eps {float} -- small constant to avoid computation instability. (default: {1e-3})

    Returns:
        np.ndarray - smoothness weights according to direction x. same dimension as `L`.
    """
    Lp = cv2.Sobel(L, cv2.CV_64F, int(x == 1), int(x == 0), ksize=1)
    T = convolve(np.ones_like(L), kernel, mode='constant')
    T = T / (np.abs(convolve(Lp, kernel, mode='constant')) + eps)
    return T / (np.abs(Lp) + eps)


def fuse_multi_exposure_images(cv2_img: np.ndarray, under_ex: np.ndarray, over_ex: np.ndarray,
                               bc: float = 1, bs: float = 1, be: float = 1):
    """perform the exposure fusion method used in the DUAL paper.

    Arguments:
        cv2_img {np.ndarray} -- input image to be enhanced.
        under_ex {np.ndarray} -- under-exposure corrected image. same dimension as `cv2_img`.
        over_ex {np.ndarray} -- over-exposure corrected image. same dimension as `cv2_img`.

    Keyword Arguments:
        bc {float} -- parameter for controlling the influence of Mertens's contrast measure. (default: {1})
        bs {float} -- parameter for controlling the influence of Mertens's saturation measure. (default: {1})
        be {float} -- parameter for controlling the influence of Mertens's well exposedness measure. (default: {1})

    Returns:
        np.ndarray -- the fused image. same dimension as `cv2_img`.
    """
    merge_mertens = cv2.createMergeMertens(bc, bs, be)
    images = [np.clip(x * 255, 0, 255).astype("uint8") for x in [cv2_img, under_ex, over_ex]]
    fused_images = merge_mertens.process(images)
    return fused_images


def refine_illumination_map_linear(L: np.ndarray, gamma: float, lambda_: float, kernel: np.ndarray, eps: float = 1e-3):
    """Refine the illumination map based on the optimization problem described in the two papers.
       This function use the sped-up solver presented in the LIME paper.

    Arguments:
        L {np.ndarray} -- the illumination map to be refined.
        gamma {float} -- gamma correction factor.
        lambda_ {float} -- coefficient to balance the terms in the optimization problem.
        kernel {np.ndarray} -- spatial affinity matrix.

    Keyword Arguments:
        eps {float} -- small constant to avoid computation instability (default: {1e-3}).

    Returns:
        np.ndarray -- refined illumination map. same shape as `L`.
    """
    # compute smoothness weights
    wx = compute_smoothness_weights(L, x=1, kernel=kernel, eps=eps)
    wy = compute_smoothness_weights(L, x=0, kernel=kernel, eps=eps)

    n, m = L.shape
    L_1d = L.copy().flatten()

    # compute the five-point spatially inhomogeneous Laplacian matrix
    row, column, data = [], [], []
    for p in range(n * m):
        diag = 0
        for q, (k, l, x) in get_sparse_neighbor(p, n, m).items():
            weight = wx[k, l] if x else wy[k, l]
            row.append(p)
            column.append(q)
            data.append(-weight)
            diag += weight
        row.append(p)
        column.append(p)
        data.append(diag)
    F = csr_matrix((data, (row, column)), shape=(n * m, n * m))

    # solve the linear system
    Id = diags([np.ones(n * m)], [0])
    A = Id + lambda_ * F
    L_refined = spsolve(csr_matrix(A), L_1d, permc_spec=None, use_umfpack=True).reshape((n, m))

    # gamma correction
    L_refined = np.clip(L_refined, eps, 1) ** gamma

    return L_refined


def correct_underexposure(cv2_img: np.ndarray, gamma: float, lambda_: float, kernel: np.ndarray, eps: float = 1e-3):
    """correct underexposudness using the retinex based algorithm presented in DUAL and LIME paper.

    Arguments:
        cv2_img {np.ndarray} -- input image to be corrected.
        gamma {float} -- gamma correction factor.
        lambda_ {float} -- coefficient to balance the terms in the optimization problem.
        kernel {np.ndarray} -- spatial affinity matrix.

    Keyword Arguments:
        eps {float} -- small constant to avoid computation instability (default: {1e-3})

    Returns:
        np.ndarray -- image underexposudness corrected. same shape as `cv2_img`.
    """

    # first estimation of the illumination map
    L = np.max(cv2_img, axis=-1)
    # illumination refinement
    L_refined = refine_illumination_map_linear(L, gamma, lambda_, kernel, eps)

    # correct image underexposure
    L_refined_3d = np.repeat(L_refined[..., None], 3, axis=-1)
    im_corrected = cv2_img / L_refined_3d
    return im_corrected

# TODO: resize image if too large, optimization take too much time



def get_sparse_neighbor(p: int, n: int, m: int):
    """Returns a dictionnary, where the keys are index of 4-neighbor of `p` in the sparse matrix,
       and values are tuples (i, j, x), where `i`, `j` are index of neighbor in the normal matrix,
       and x is the direction of neighbor.

    Arguments:
        p {int} -- index in the sparse matrix.
        n {int} -- number of rows in the original matrix (non sparse).
        m {int} -- number of columns in the original matrix.

    Returns:
        dict -- dictionnary containing indices of 4-neighbors of `p`.
    """
    i, j = p // m, p % m
    d = {}
    if i - 1 >= 0:
        d[(i - 1) * m + j] = (i - 1, j, 0)
    if i + 1 < n:
        d[(i + 1) * m + j] = (i + 1, j, 0)
    if j - 1 >= 0:
        d[i * m + j - 1] = (i, j - 1, 1)
    if j + 1 < m:
        d[i * m + j + 1] = (i, j + 1, 1)
    return d   




################################
# Data Plotting

def get_cv2_plot(data_dict):
    """
    Converts a dict {label: (times, values)} into a BGR OpenCV image.
    """
    # Create the plot using Matplotlib
    fig, ax = plt.subplots(figsize=(8, 6))
    
    for label, (times, values) in data_dict.items():
        ax.plot(times, values, label=label) #

    ax.set_title("Time Series Data")
    ax.set_xlabel("Time")
    ax.set_ylabel("Values")
    ax.legend()
    ax.grid(True)

    # Save the plot to an in-memory buffer
    buf = io.BytesIO()
    plt.savefig(buf, format='png', dpi=100)
    buf.seek(0)

    # Convert buffer to a numpy array for OpenCV
    file_bytes = np.asarray(bytearray(buf.read()), dtype=np.uint8)
    img = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)

    # Clean up Matplotlib figure to free memory
    plt.close(fig)
    
    return img


