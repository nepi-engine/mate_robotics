/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi rui (nepi_apps) repo
# (see https://github.com/nepi-engine/nepi_apps)
#
# License: NEPI RUI repo source-code and NEPI Images that use this source-code
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
 */

import React, { Component } from "react"
import { observer, inject } from "mobx-react"

import Section from "./Section"
import { Columns, Column } from "./Columns"
import Label from "./Label"
import Input from "./Input"
import Button, { ButtonMenu } from "./Button"
import Styles from "./Styles"
import AsyncToggle from "./AsyncToggle"
import BooleanIndicator from "./BooleanIndicator"
import { SliderAdjustment } from "./AdjustmentWidgets"

import NepiIFImageViewer from "./Nepi_IF_ImageViewer"
import NepiIFConnectRBX from "./Nepi_IF_ConnectRBX"
import NepiIFConnectData from "./Nepi_IF_ConnectData"
import NepiIFControls from "./Nepi_IF_Controls"
import NepiIFConfig from "./Nepi_IF_Config"

import { setElementStyleModified, clearElementStyleModified } from "./Utilities"

function round(value, decimals = 0) {
  return Number(value).toFixed(decimals)
}

@inject("ros")
@observer

// Auto Move app main panel. Left column is the overlay image viewer, right
// column is the app panel: the robot and image selectors, a readout of which
// companion topics were resolved, the overlay and goto controls, and the app's
// ControlsIF box.
//
// This page binds to ONE app node, not to a manager list. The status topic is
// <app>/status carrying nepi_app_auto_move/NepiAppAutoMoveStatus, and every
// command topic hangs off that same <app> namespace -- the namespace AutoMoveIF
// registers its subscribers on.
//
// The two selectors are the standard connect components pointed at the connect
// namespaces the node owns:
//   Robot -> <app>/rbx_connect    (ConnectRBXDeviceIF)
//   Image -> <app>/image_connect  (ConnectImageIF)
// Both names are declared node-side as explicit connect_name arguments so the
// binding is greppable from both sides; the names held in state below must match
// those node-side names character for character. Each selector runs with
// show_data={false} show_controls={false} -- this page renders its own viewer
// and its own goto controls, so the connect components are used purely as
// source selectors.
//
// The viewer in the left column is NOT mounted on the selected image. It is
// mounted on status.overlay_image_topic, the composited frame the app's image
// pub node publishes, and its clicks are routed to status.mouse_event_topic via
// the viewer's existing mouse_event_topic prop -- no click handling is added to
// Nepi_IF_ImageViewer itself.
class NepiAppAutoMove extends Component {

  constructor(props) {
    super(props)

    this.state = {
      appName: "app_auto_move",
      appNamespace: null,

      // Connect names, one per connect IF the node instantiates. Held here
      // rather than inlined at each call site so the strings that must match
      // the node read as one list.
      rbxConnectName: "rbx_connect",
      imageConnectName: "image_connect",

      status_msg: null,
      connected: false,

      // Editable goto boxes. Each mirrors the matching status field until the
      // operator types in it, at which point the box holds the pending edit
      // until Enter sends it. componentDidUpdate resets a box when the status
      // value behind it moves -- which is what a new click does.
      goto_x: "",
      goto_y: "",
      goto_z: "",
      max_move: "",

      // Velocity mode boxes. Same mirroring contract as the position boxes
      // above: each follows its status field until the operator types in it.
      move_speed: "",
      goto_vx: "",
      goto_vy: "",
      goto_yaw_rate: "",
      goto_duration: "",

      statusListener: null,
      needs_update: true,
    }

    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.getConnectNamespace = this.getConnectNamespace.bind(this)
    this.getControlsNamespace = this.getControlsNamespace.bind(this)

    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)

    this.onUpdateGotoText = this.onUpdateGotoText.bind(this)
    this.onKeyGotoText = this.onKeyGotoText.bind(this)

    this.renderSelectors = this.renderSelectors.bind(this)
    this.renderSourceStatus = this.renderSourceStatus.bind(this)
    this.renderControls = this.renderControls.bind(this)
    this.renderGoto = this.renderGoto.bind(this)
    this.renderAppControls = this.renderAppControls.bind(this)
    this.renderConfig = this.renderConfig.bind(this)
    this.renderImageViewer = this.renderImageViewer.bind(this)
  }

  getBaseNamespace() {
    const { namespacePrefix, deviceId } = this.props.ros
    if (namespacePrefix !== null && deviceId !== null) {
      return "/" + namespacePrefix + "/" + deviceId
    }
    return null
  }

  getAppNamespace() {
    const base = this.getBaseNamespace()
    if (base !== null) {
      return base + "/" + this.state.appName
    }
    return null
  }

  // Connect namespace a Nepi_IF_Connect* component subscribes to, i.e.
  // <app>/<connect_name>. The node publishes both fully qualified on its status
  // message, so those are preferred and this join is the fallback for the window
  // before the first status arrives.
  getConnectNamespace(connectName) {
    const status_msg = this.state.status_msg
    if (status_msg != null) {
      if (connectName === this.state.rbxConnectName && status_msg.rbx_connect_namespace) {
        return status_msg.rbx_connect_namespace
      }
      if (connectName === this.state.imageConnectName && status_msg.image_connect_namespace) {
        return status_msg.image_connect_namespace
      }
    }
    const appNamespace = this.getAppNamespace()
    return (appNamespace !== null) ? appNamespace + "/" + connectName : null
  }

  // Namespace of this app's ControlsIF, taken from the status message, which
  // publishes it fully qualified. A relative namespace would resolve at the
  // global root once a RUI component appends '/status' and hands it to
  // rosbridge -- see AutoMoveIF.get_controls_namespace().
  getControlsNamespace() {
    const status_msg = this.state.status_msg
    if (status_msg != null && status_msg.controls_namespace) {
      return status_msg.controls_namespace
    }
    const appNamespace = this.getAppNamespace()
    return (appNamespace != null) ? appNamespace + "/controls" : null
  }

  statusListener(message) {
    this.setState({
      status_msg: message,
      connected: true,
    })
  }

  updateStatusListener(namespace) {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
      this.setState({ statusListener: null, status_msg: null })
    }
    if (namespace != null && namespace.indexOf('null') === -1) {
      const statusNamespace = namespace + '/status'
      var statusListener = this.props.ros.setupStatusListener(
        statusNamespace,
        "nepi_app_auto_move/NepiAppAutoMoveStatus",
        this.statusListener
      )
      this.setState({ statusListener: statusListener })
    }
    this.setState({ appNamespace: namespace, needs_update: false })
  }

  componentDidMount() {
    this.setState({ needs_update: true })
  }

  componentDidUpdate(prevProps, prevState) {
    const namespace = this.getAppNamespace()
    if ((namespace != null && namespace !== this.state.appNamespace) || this.state.needs_update === true) {
      this.updateStatusListener(namespace)
    }

    // Reset each editable box when the value behind it changes on the wire. A
    // click rewrites all three goto values, and the boxes have to follow it --
    // the whole point of click to go is that the operator sees where the click
    // landed.
    const status_msg = this.state.status_msg
    const prev_status_msg = prevState.status_msg
    if (status_msg != null) {
      if (prev_status_msg == null || prev_status_msg.goto_x_m !== status_msg.goto_x_m) {
        this.setState({ goto_x: round(status_msg.goto_x_m, 2) })
      }
      if (prev_status_msg == null || prev_status_msg.goto_y_m !== status_msg.goto_y_m) {
        this.setState({ goto_y: round(status_msg.goto_y_m, 2) })
      }
      if (prev_status_msg == null || prev_status_msg.goto_z_m !== status_msg.goto_z_m) {
        this.setState({ goto_z: round(status_msg.goto_z_m, 2) })
      }
      if (prev_status_msg == null || prev_status_msg.max_move_m !== status_msg.max_move_m) {
        this.setState({ max_move: round(status_msg.max_move_m, 2) })
      }
      if (prev_status_msg == null || prev_status_msg.move_speed_mps !== status_msg.move_speed_mps) {
        this.setState({ move_speed: round(status_msg.move_speed_mps, 2) })
      }
      if (prev_status_msg == null || prev_status_msg.goto_x_mps !== status_msg.goto_x_mps) {
        this.setState({ goto_vx: round(status_msg.goto_x_mps, 2) })
      }
      if (prev_status_msg == null || prev_status_msg.goto_y_mps !== status_msg.goto_y_mps) {
        this.setState({ goto_vy: round(status_msg.goto_y_mps, 2) })
      }
      if (prev_status_msg == null || prev_status_msg.goto_yaw_degps !== status_msg.goto_yaw_degps) {
        this.setState({ goto_yaw_rate: round(status_msg.goto_yaw_degps, 2) })
      }
      if (prev_status_msg == null || prev_status_msg.goto_duration_s !== status_msg.goto_duration_s) {
        this.setState({ goto_duration: round(status_msg.goto_duration_s, 2) })
      }
    }
  }

  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
    this.setState({
      status_msg: null,
      connected: false,
      statusListener: null,
    })
  }

  //////////////////////////
  // Editable goto boxes -- the PTX controls pattern

  onUpdateGotoText(e, stateKey, elementId) {
    const el = document.getElementById(elementId)
    setElementStyleModified(el)
    var update = {}
    update[stateKey] = e.target.value
    this.setState(update)
  }

  onKeyGotoText(e, elementId, topic) {
    if (e.key !== 'Enter') {
      return
    }
    const el = document.getElementById(elementId)
    clearElementStyleModified(el)
    const value = parseFloat(el.value)
    if (isNaN(value)) {
      return
    }
    this.props.ros.sendFloatMsg(topic, value)
  }

  //////////////////////////
  // Right column

  renderSelectors() {
    const divider = <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

    return (
      <Section title={"Connections"}>

        <NepiIFConnectRBX
          namespace={this.getConnectNamespace(this.state.rbxConnectName)}
          title={"Robot"}
          show_selector={true}
          show_data={false}
          show_controls={false}
          show_connect_header={true}
          make_section={false}
        />

        {divider}

        <NepiIFConnectData
          namespace={this.getConnectNamespace(this.state.imageConnectName)}
          title={"Image"}
          show_selector={true}
          show_data={false}
          show_connect_header={true}
          make_section={false}
        />

      </Section>
    )
  }

  // What the app resolved from the selected image, and whether each companion
  // is live. A false flag is not an error -- the app keeps running and the
  // feature that needed that topic goes inert -- so each row reports the topic
  // it looked for alongside its indicator.
  renderSourceStatus() {
    const status_msg = this.state.status_msg
    if (status_msg == null) {
      return null
    }

    const rows = [
      { title: "Depth Map", topic: status_msg.depth_map_topic, found: status_msg.depth_map_found },
      { title: "Depth Map Image", topic: status_msg.depth_map_image_topic, found: status_msg.depth_map_image_found },
      { title: "Objects", topic: status_msg.objects_topic, found: status_msg.objects_found },
      { title: "Targets", topic: status_msg.targets_topic, found: status_msg.targets_found },
      { title: "Obstacles", topic: status_msg.obstacles_topic, found: status_msg.obstacles_found },
    ]

    return (
      <Section title={"Sources"}>

        <div style={{ display: 'flex' }}>
          <div style={{ width: '50%' }}>
            <Label title={"Robot Connected"}>
              <BooleanIndicator value={status_msg.rbx_connected} />
            </Label>
          </div>
          <div style={{ width: '50%' }}>
            <Label title={"Robot Ready"}>
              <BooleanIndicator value={status_msg.rbx_ready} />
            </Label>
          </div>
        </div>

        <Label title={"Image Connected"}>
          <BooleanIndicator value={status_msg.image_connected} />
        </Label>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        {rows.map((row) =>
          <Label title={row.title}>
            <BooleanIndicator value={row.found === true} />
          </Label>
        )}

        <pre style={{ height: "100px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
        {rows.map((row) => row.title + ": " + ((row.topic !== '') ? row.topic : "not resolved") + "\n").join('')}
        </pre>

      </Section>
    )
  }

  renderControls() {
    const { sendBoolMsg } = this.props.ros
    const status_msg = this.state.status_msg
    if (status_msg == null) {
      return null
    }
    const appNamespace = this.getAppNamespace()

    const show_depth_map = status_msg.show_depth_map_enabled
    const show_objects = status_msg.show_objects_enabled
    const show_targets = status_msg.show_targets_enabled
    const show_obstacles = status_msg.show_obstacles_enabled
    const show_crosshair = status_msg.show_crosshair_enabled

    // 0.0 fully opaque through 1.0 invisible on the wire; the slider below runs
    // 0-100 with scaled 0.01, the RUI's standard ratio slider shape.
    const depth_map_transparency = status_msg.depth_map_transparency

    return (
      <Section title={"Overlay"}>

        <Label title="Show Depth Map">
          <AsyncToggle
            checked={show_depth_map === true}
            onClick={() => sendBoolMsg(appNamespace + "/set_show_depth_map", show_depth_map === false)}>
          </AsyncToggle>
        </Label>

        {/* The transparency slider sits under its own show toggle and is hidden
            while the layer is off -- with nothing drawn there is nothing for it
            to adjust. */}
        <div hidden={show_depth_map === false}>
          <SliderAdjustment
            title={"Depth Map Transparency"}
            msgType={"std_msgs/Float32"}
            adjustment={depth_map_transparency}
            topic={appNamespace + "/set_depth_map_transparency"}
            scaled={0.01}
            min={0}
            max={100}
            disabled={false}
            tooltip={"Sets depth map overlay transparency, 100% is invisible"}
            unit={"%"}
          />
        </div>

        <Label title="Show Objects">
          <AsyncToggle
            checked={show_objects === true}
            onClick={() => sendBoolMsg(appNamespace + "/set_show_objects", show_objects === false)}>
          </AsyncToggle>
        </Label>

        <Label title="Show Targets">
          <AsyncToggle
            checked={show_targets === true}
            onClick={() => sendBoolMsg(appNamespace + "/set_show_targets", show_targets === false)}>
          </AsyncToggle>
        </Label>

        <Label title="Show Obstacles">
          <AsyncToggle
            checked={show_obstacles === true}
            onClick={() => sendBoolMsg(appNamespace + "/set_show_obstacles", show_obstacles === false)}>
          </AsyncToggle>
        </Label>

        <Label title="Show Crosshair">
          <AsyncToggle
            checked={show_crosshair === true}
            onClick={() => sendBoolMsg(appNamespace + "/set_show_crosshair", show_crosshair === false)}>
          </AsyncToggle>
        </Label>

      </Section>
    )
  }

  // The goto section. The offset boxes display what the node computed from the
  // last click and are directly editable -- an operator can nudge a value the
  // click got nearly right instead of hunting for a better pixel.
  //
  // Two modes, one click. The toggle at the top picks what a Goto issues: a
  // distance the robot closes on, or a body-frame velocity held for a duration.
  // The click resolves to both either way, so switching modes never invalidates
  // a click. Max Move governs the click in both modes, so its clamp indicator
  // stays visible in both.
  renderGoto() {
    const { sendTriggerMsg, sendBoolMsg } = this.props.ros
    const status_msg = this.state.status_msg
    if (status_msg == null) {
      return null
    }
    const appNamespace = this.getAppNamespace()

    const goto_state = status_msg.goto_state
    const goto_running = (goto_state === 'planning' || goto_state === 'moving')

    const velocity_mode = (status_msg.goto_velocity_enabled === true)
    const robot_has_velocity = (status_msg.rbx_has_goto_velocity === true)
    const has_max_speed = (status_msg.robot_max_speed_mps > 0)

    return (
      <Section title={"Goto"}>

        <Label title={"Click the image to set a destination"} />

        <Label title="Velocity Mode">
          <AsyncToggle
            checked={velocity_mode}
            onClick={() => sendBoolMsg(appNamespace + "/set_goto_velocity_enabled", velocity_mode === false)}>
          </AsyncToggle>
        </Label>

        {/* Says what is wrong and nothing else. The toggle stays usable -- the
            operator may be selecting the robot next -- but a Goto in this state
            is refused by the node rather than published into nothing. */}
        <div hidden={!(velocity_mode === true && robot_has_velocity === false)}>
          <Label title={"The connected robot does not support velocity moves"} />
        </div>

        {/* Position mode: exactly what was rendered before velocity mode existed. */}
        <div hidden={velocity_mode === true}>

          <Label title={"X Forward (m)"}>
            <Input
              id={"AutoMoveGotoX"}
              value={this.state.goto_x}
              onChange={(e) => this.onUpdateGotoText(e, 'goto_x', 'AutoMoveGotoX')}
              onKeyDown={(e) => this.onKeyGotoText(e, 'AutoMoveGotoX', appNamespace + "/set_goto_x")}
            />
          </Label>

          <Label title={"Y Left (m)"}>
            <Input
              id={"AutoMoveGotoY"}
              value={this.state.goto_y}
              onChange={(e) => this.onUpdateGotoText(e, 'goto_y', 'AutoMoveGotoY')}
              onKeyDown={(e) => this.onKeyGotoText(e, 'AutoMoveGotoY', appNamespace + "/set_goto_y")}
            />
          </Label>

          <Label title={"Z Up (m)"}>
            <Input
              id={"AutoMoveGotoZ"}
              value={this.state.goto_z}
              onChange={(e) => this.onUpdateGotoText(e, 'goto_z', 'AutoMoveGotoZ')}
              onKeyDown={(e) => this.onKeyGotoText(e, 'AutoMoveGotoZ', appNamespace + "/set_goto_z")}
            />
          </Label>

          <Label title={"Max Move (m)"}>
            <Input
              id={"AutoMoveMaxMove"}
              value={this.state.max_move}
              onChange={(e) => this.onUpdateGotoText(e, 'max_move', 'AutoMoveMaxMove')}
              onKeyDown={(e) => this.onKeyGotoText(e, 'AutoMoveMaxMove', appNamespace + "/set_max_move")}
            />
          </Label>

        </div>

        {/* Velocity mode. Device Speed sits FIRST because the four boxes under
            it are computed from it: the click distance divided by this speed is
            the duration, and the velocities follow from that duration.

            A robot that reports its max speed gets a ratio slider against that
            max. One that does not (0.0 on the wire) falls back to the hand
            entered m/s box. A new speed applies from the next click. */}
        <div hidden={velocity_mode === false}>

          <div hidden={has_max_speed === false}>
            <SliderAdjustment
              title={"Device Speed"}
              msgType={"std_msgs/Float32"}
              adjustment={status_msg.move_speed_ratio}
              topic={appNamespace + "/set_move_speed_ratio"}
              scaled={0.01}
              min={5}
              max={100}
              disabled={false}
              tooltip={"Velocity move speed as a percent of the robot's reported max speed"}
              unit={"%"}
            />
            <Label title={"Speed (m/s)"}>
              {round(status_msg.robot_max_speed_mps * status_msg.move_speed_ratio, 2) +
               " of " + round(status_msg.robot_max_speed_mps, 2) + " max"}
            </Label>
          </div>

          <div hidden={has_max_speed === true}>
            <Label title={"Device Speed (m/s)"}>
              <Input
                id={"AutoMoveMoveSpeed"}
                value={this.state.move_speed}
                onChange={(e) => this.onUpdateGotoText(e, 'move_speed', 'AutoMoveMoveSpeed')}
                onKeyDown={(e) => this.onKeyGotoText(e, 'AutoMoveMoveSpeed', appNamespace + "/set_move_speed")}
              />
            </Label>
            <Label title={"Robot reports no max speed"} />
          </div>

          <Label title={"X Velocity (m/s)"}>
            <Input
              id={"AutoMoveGotoVx"}
              value={this.state.goto_vx}
              onChange={(e) => this.onUpdateGotoText(e, 'goto_vx', 'AutoMoveGotoVx')}
              onKeyDown={(e) => this.onKeyGotoText(e, 'AutoMoveGotoVx', appNamespace + "/set_goto_vx")}
            />
          </Label>

          <Label title={"Y Velocity (m/s)"}>
            <Input
              id={"AutoMoveGotoVy"}
              value={this.state.goto_vy}
              onChange={(e) => this.onUpdateGotoText(e, 'goto_vy', 'AutoMoveGotoVy')}
              onKeyDown={(e) => this.onKeyGotoText(e, 'AutoMoveGotoVy', appNamespace + "/set_goto_vy")}
            />
          </Label>

          <Label title={"Rotational Velocity (deg/s)"}>
            <Input
              id={"AutoMoveGotoYawRate"}
              value={this.state.goto_yaw_rate}
              onChange={(e) => this.onUpdateGotoText(e, 'goto_yaw_rate', 'AutoMoveGotoYawRate')}
              onKeyDown={(e) => this.onKeyGotoText(e, 'AutoMoveGotoYawRate', appNamespace + "/set_goto_yaw_rate")}
            />
          </Label>

          <Label title={"Seconds"}>
            <Input
              id={"AutoMoveGotoDuration"}
              value={this.state.goto_duration}
              onChange={(e) => this.onUpdateGotoText(e, 'goto_duration', 'AutoMoveGotoDuration')}
              onKeyDown={(e) => this.onKeyGotoText(e, 'AutoMoveGotoDuration', appNamespace + "/set_goto_duration")}
            />
          </Label>

        </div>

        {/* Visible in both modes: the max_move_m clamp governs the click, and
            in velocity mode the clamped distance is what the duration and the
            velocities are derived from. */}
        <Label title={"Clamped to Max Move"}>
          <BooleanIndicator value={status_msg.goto_clamped === true} />
        </Label>

        <ButtonMenu>
          <Button onClick={() => sendTriggerMsg(appNamespace + "/goto_trigger")}>
            {"Goto"}
          </Button>
          <Button onClick={() => sendTriggerMsg(appNamespace + "/goto_cancel")}>
            {"Cancel"}
          </Button>
        </ButtonMenu>

        <pre style={{ height: "80px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
        {"State: " + goto_state +
         ((goto_running === true && status_msg.goto_step_count > 0) ?
            "  (step " + (status_msg.goto_step + 1) + " of " + status_msg.goto_step_count + ")" : "") +
         "\n" + status_msg.goto_msg +
         "\n" + status_msg.click_msg}
        </pre>

      </Section>
    )
  }

  // The app's own control set, rendered by the shared Nepi_IF_Controls against
  // the fully-qualified namespace the status message reports.
  renderAppControls() {
    const controls_namespace = this.getControlsNamespace()
    if (controls_namespace == null) {
      return null
    }
    return (
      <Section title={"Move Controls"}>

        <NepiIFControls
          namespace={controls_namespace}
          make_section={false}
        />

      </Section>
    )
  }

  renderConfig() {
    return (
      <NepiIFConfig
        namespace={this.getAppNamespace()}
        title={"Nepi_IF_Config"}
      />
    )
  }

  //////////////////////////
  // Left column

  // The composited overlay the app's image pub node publishes -- source frame,
  // depth map blend, obstacles, objects, targets and the click crosshair, in
  // that order. Mounted only once the topic is actually advertised, so the
  // viewer shows a waiting title instead of a broken image.
  //
  // mouse_event_topic is what routes clicks to the app node. Without it the
  // viewer would publish to <image topic>/mouse_event, which belongs to the
  // image source, not to this app.
  renderImageViewer() {
    const { imageTopics } = this.props.ros
    const status_msg = this.state.status_msg

    const overlay_topic = (status_msg != null && status_msg.overlay_image_topic) ? status_msg.overlay_image_topic : 'None'
    const mouse_event_topic = (status_msg != null && status_msg.mouse_event_topic) ? status_msg.mouse_event_topic : null

    const publishing = imageTopics.indexOf(overlay_topic) !== -1
    const image_topic = (publishing === true && this.state.connected === true) ? overlay_topic : 'None'
    const image_topic_text = (overlay_topic === 'None') ? 'No Image Selected' :
      publishing ? 'Auto Move' : 'Waiting for image to publish'

    return (
      <Columns>
        <Column>

          <NepiIFImageViewer
            image_topic={image_topic}
            title={image_topic_text}
            mouse_event_topic={mouse_event_topic}
            show_res_orient={false}
          />

        </Column>
      </Columns>
    )
  }

  // Standard NEPI device-panel split: the viewer in the left ~75%, a small
  // gutter, and the app panel in the right ~23% column.
  renderBody() {
    return (
      <div style={{ display: 'flex' }}>

        <div style={{ width: "75%" }}>
          {this.renderImageViewer()}
        </div>

        <div style={{ width: '2%' }}>
          {}
        </div>

        <div style={{ width: "23%" }}>
          {this.renderSelectors()}
          {this.renderSourceStatus()}
          {this.renderControls()}
          {this.renderGoto()}
          {this.renderAppControls()}
          {this.renderConfig()}
        </div>

      </div>
    )
  }

  render() {
    const make_section = (this.props.make_section !== undefined) ? this.props.make_section : true

    if (make_section === false) {
      return (
        <Columns>
          <Column>
            {this.renderBody()}
          </Column>
        </Columns>
      )
    } else {
      return (
        <Section>
          {this.renderBody()}
        </Section>
      )
    }
  }
}

export default NepiAppAutoMove
