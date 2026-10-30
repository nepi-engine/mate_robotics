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

/*
 * NEPI APP ROBOT_STAB -- RUI PAGE
 * ---------------------------------------------------------------------------
 * This file is installed FLAT into the RUI source tree
 * (.../rui-app/src/) by the app's CMakeLists, so its basename shares one
 * global namespace with every other app's rui/ files -- a duplicate basename
 * silently overwrites. Keep it unique.
 *
 * Registration is GENERATED, never hand-written. build_nepi_rui.sh walks
 * src/apps/*.yaml, reads RUI_DICT.rui_main_file and RUI_DICT.rui_main_class,
 * and sed-injects an import line and a ["<class>", <class>] classMap entry
 * into Nepi_IF_Apps.js. Never hand-edit Nepi_IF_Apps.js or NepiApps.js.
 * rui_main_class is used as BOTH the import binding and the map key, so two
 * apps declaring the same class name break the build.
 *
 * Layout is the 75 / 2 / 23 split NepiAppIDXConnect.js uses:
 *   - left 75%:  the image viewer. The camera and data product are picked on
 *     this page (local state, not persisted); see renderSelection().
 *   - right 23%: what the node REPORTS (this app's own status message), the
 *     RBX / Targets / Obstacles selector rows, then what the operator ADJUSTS (the shared
 *     Nepi_IF_Controls renderer, fed by the node's ControlsStatus -- do not
 *     hand-write a widget per value) and the config box.
 *
 * The namespace prop handed to Nepi_IF_Controls is the CONTROLS namespace
 * (<app>/controls), not the app namespace: the component appends '/status'
 * itself and its children append '/update_control'. Passing the app namespace
 * is the single most common way to get an empty control box -- a heading that
 * renders and then never fills in.
 *
 * Each Nepi_IF_Connect<X> row is handed its CONNECT namespace
 * (<app>/<connect_name>), which the node's Connect*IF owns end to end:
 * discovery, the persisted selection, ConnectIFStatus and select_topic. The
 * Obstacles row is the exception: ConnectObstaclesIF is not a ConnectNodeIF and
 * publishes no ConnectIFStatus, so that row is built inline here (as the
 * WPILib IF page does) and publishes the pick to <app>/set_obstacles_namespace.
 */

import React, { Component } from "react"
import { observer, inject } from "mobx-react"

import Section from "./Section"
import { Columns, Column } from "./Columns"
import Label from "./Label"
import Input from "./Input"
import Select, { Option } from "./Select"
import BooleanIndicator from "./BooleanIndicator"
import Styles from "./Styles"

import NepiIFImageViewer from "./Nepi_IF_ImageViewer"
import NepiIFConnectRBX from "./Nepi_IF_ConnectRBX"
import NepiIFConnectTargets from "./Nepi_IF_ConnectTargets"

import NepiIFControls from "./Nepi_IF_Controls"
import NepiIFConfig from "./Nepi_IF_Config"

// Must match CONTROLS_NAME in scripts/robot_stab_app_node.py. A ControlsIF is
// always a direct child of the node namespace, so the node and this page each
// derive the same name independently.
const CONTROLS_NAME = "controls"

// Must match CONNECT_NAME in nepi_api/connect_device_if_rbx.py and
// TARGETS_CONNECT_NAME in nepi_api/connect_process_if_targets.py. The node
// constructs both connects with their default names.
const RBX_CONNECT_NAME = "rbx_connect"
const TARGETS_CONNECT_NAME = "targets_connect"

// Message type every obstacles app publishes on <app>/obstacles/status. The
// Obstacles row's option list is every topic of this type currently on the
// system, with the trailing /obstacles/status stripped back off to give the app
// namespace ConnectObstaclesIF takes. Same discovery as the WPILib IF page.
const OBSTACLES_STATUS_TYPE = "nepi_app_obstacles/ObstaclesStatus"
const OBSTACLES_STATUS_SUFFIX = "/obstacles/status"

// Unselected state of the Obstacles row, matching NONE_NAMESPACE in the node.
const OBSTACLES_NONE = "None"

@inject("ros")
@observer

// RobotStab Application page
class NepiAppRobotStab extends Component {

  constructor(props) {
    super(props)

    this.state = {
      // Must match APP_DICT.node_name in params/robot_stab_app_params.yaml.
      // apps_mgr launches the node under the yaml's node_name, so the yaml --
      // not DEFAULT_NODE_NAME in the script -- is what this has to agree with.
      appName: "app_robot_stab",
      appNamespace: null,

      // Read-only status fields — mirror NepiAppRobotStabStatus.msg. The
      // rbx/targets *_connected fields are not repeated here: each connect row
      // below renders its own Connected indicator.
      enabled: false,
      selected_option: "None",
      value: 0.0,
      obstacles_connected: false,
      selected_obstacles_namespace: null,

      // Operator's obstacles app selection, held locally so the row shows the
      // pick straight away in the window before the node's next status message
      // reports it back. Once status arrives, selected_obstacles_namespace is
      // authoritative.
      obstacles_namespace: OBSTACLES_NONE,

      statusListener: null,
      connected: false,

      // Selected camera (<device>/idx), picked on this page from the IDX
      // devices the ros store already tracks. Local to the page.
      selected_topic: 'None',

      // Image viewer data product selection, local to this page
      data_topic: 'None',
      data_product: 'None',
    }

    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.getControlsNamespace = this.getControlsNamespace.bind(this)
    this.getConnectNamespace = this.getConnectNamespace.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.getObstaclesNamespaces = this.getObstaclesNamespaces.bind(this)
    this.getSelectedObstaclesNamespace = this.getSelectedObstaclesNamespace.bind(this)
    this.onObstaclesSelected = this.onObstaclesSelected.bind(this)

    this.createCameraOptions = this.createCameraOptions.bind(this)
    this.onCameraSelected = this.onCameraSelected.bind(this)

    this.createDataProductOptions = this.createDataProductOptions.bind(this)
    this.onDataProductSelected = this.onDataProductSelected.bind(this)
    this.renderDataProductSelector = this.renderDataProductSelector.bind(this)
    this.renderSelection = this.renderSelection.bind(this)
    this.findImageTopic = this.findImageTopic.bind(this)
    this.renderImageViewer = this.renderImageViewer.bind(this)

    this.renderStatus = this.renderStatus.bind(this)
    this.renderConnections = this.renderConnections.bind(this)
    this.renderControls = this.renderControls.bind(this)
    this.renderConfig = this.renderConfig.bind(this)
    this.renderPanel = this.renderPanel.bind(this)
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

  getControlsNamespace() {
    const appNamespace = this.getAppNamespace()
    if (appNamespace !== null) {
      return appNamespace + "/" + CONTROLS_NAME
    }
    return null
  }

  getConnectNamespace(connectName) {
    const appNamespace = this.getAppNamespace()
    if (appNamespace !== null) {
      return appNamespace + "/" + connectName
    }
    return null
  }

  statusListener(message) {
    this.setState({
      enabled: message.enabled,
      selected_option: message.selected_option,
      value: message.value,
      obstacles_connected: message.obstacles_connected,
      selected_obstacles_namespace: message.selected_obstacles_namespace,
      connected: true,
    })
  }

  updateStatusListener(namespace) {
    const statusNamespace = namespace + '/status'
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
    var statusListener = this.props.ros.setupStatusListener(
      statusNamespace,
      "nepi_app_robot_stab/NepiAppRobotStabStatus",
      this.statusListener
    )
    this.setState({
      appNamespace: namespace,
      statusListener: statusListener,
    })
  }

  componentDidMount() {
    const namespace = this.getAppNamespace()
    if (namespace !== null) {
      this.updateStatusListener(namespace)
    }
  }

  componentDidUpdate(prevProps, prevState) {
    const namespace = this.getAppNamespace()
    const namespace_updated = (this.state.appNamespace !== namespace && namespace !== null)
    if (namespace_updated) {
      if (namespace.indexOf('null') === -1) {
        this.updateStatusListener(namespace)
      }
    }
  }

  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
  }

  // Live list of obstacles app namespaces, read off the topic/type lists the
  // ros store already keeps for the whole system. An obstacles app is any node
  // publishing <app>/obstacles/status as an ObstaclesStatus.
  getObstaclesNamespaces() {
    const { topicNames, topicTypes } = this.props.ros
    var namespaces = []
    if (topicNames == null || topicTypes == null) {
      return namespaces
    }
    for (var i = 0; i < topicNames.length; i++) {
      if (topicTypes[i] === OBSTACLES_STATUS_TYPE &&
          topicNames[i].endsWith(OBSTACLES_STATUS_SUFFIX)) {
        namespaces.push(topicNames[i].slice(0, -OBSTACLES_STATUS_SUFFIX.length))
      }
    }
    namespaces.sort()
    return namespaces
  }

  // What the Obstacles row shows. The node's status is authoritative once it
  // arrives, with the local pick covering the window before the first status.
  getSelectedObstaclesNamespace() {
    const selected = this.state.selected_obstacles_namespace
    if (selected != null && selected !== '') {
      return selected
    }
    return this.state.obstacles_namespace
  }

  onObstaclesSelected(event) {
    const appNamespace = this.getAppNamespace()
    const value = event.target.value
    this.setState({ obstacles_namespace: value })
    if (appNamespace != null) {
      this.props.ros.sendStringMsg(appNamespace + '/set_obstacles_namespace', value)
    }
  }

  // Camera options: every IDX device namespace the ros store tracks.
  createCameraOptions() {
    const { idxDevices } = this.props.ros
    const namespaces = (idxDevices != null) ? Object.keys(idxDevices).sort() : []
    var items = []
    items.push(<Option value={"None"}>{"None"}</Option>)
    for (var i = 0; i < namespaces.length; i++) {
      items.push(<Option value={namespaces[i]}>{namespaces[i].split('/idx')[0].split('/').pop()}</Option>)
    }
    return items
  }

  // Clears the data product selection whenever the selected camera changes so
  // the image viewer re-resolves.
  onCameraSelected(event) {
    const value = event.target.value
    if (value !== this.state.selected_topic) {
      this.setState({
        selected_topic: value,
        data_topic: 'None',
        data_product: 'None'
      })
    }
  }

  // Function for creating data product options for Select input. The IDX
  // selected_topic is already the device's '<device>/idx' namespace, so data
  // products sit directly beneath it -- never re-insert 'idx'.
  createDataProductOptions() {
    const namespace = (this.state.selected_topic !== null) ? this.state.selected_topic : 'None'
    const capabilities = this.props.ros.idxDevices[namespace]
    const data_products = capabilities ? capabilities.data_products : []

    var items = []
    var data_product
    var data_topic

    for (var i = 0; i < data_products.length; i++) {
      data_product = data_products[i]
      data_topic = namespace + '/' + data_product
      items.push(<Option value={data_topic}>{data_product}</Option>)
    }

    const sel_data_topic = this.state.data_topic
    if (items.length === 0) {
      items.push(<Option value={"None"}>{"None"}</Option>)
      if (sel_data_topic !== 'None') {
        this.setState({
          data_topic: "None",
          data_product: "None",
        })
      }
    }
    else if (sel_data_topic === 'None' || sel_data_topic == null) {
      this.setState({
        data_topic: namespace + '/' + data_products[0],
        data_product: data_products[0],
      })
    }

    return items
  }

  // Handler for data product selection
  onDataProductSelected(event) {
    const index = event.nativeEvent.target.selectedIndex
    const text = event.nativeEvent.target[index].text
    const value = event.target.value

    this.setState({
      data_topic: value,
      data_product: text,
    })
  }

  renderDataProductSelector() {
    const data_topic = this.state.data_topic

    return (

      <React.Fragment>

        <div align={"left"} textAlign={"left"}>
          <Label title={"Data Product"}>
            <Select
              id="topicSelect"
              onChange={this.onDataProductSelected}
              value={data_topic}
            >
              {this.createDataProductOptions()}
            </Select>
          </Label>
        </div>

      </React.Fragment>
    )
  }

  // Camera and data product selection. Local to this page and drives the
  // image viewer only. The node has no camera connect: the targets connect's
  // ConnectIFStatus names no image topic for the viewer to follow, so the
  // camera is picked here, with the same data-product pattern as before.
  renderSelection() {
    const device_selected = (this.state.selected_topic !== null && this.state.selected_topic !== 'None')

    return (
      <Section title={"Selection"}>

        <div align={"left"} textAlign={"left"}>
          <Label title={"Camera"}>
            <Select
              id="cameraSelect"
              onChange={this.onCameraSelected}
              value={this.state.selected_topic}
            >
              {this.createCameraOptions()}
            </Select>
          </Label>
        </div>

        {(device_selected === true) ?
          this.renderDataProductSelector()
          : null}

      </Section>
    )
  }

  findImageTopic(data_product) {
    const namespace = (this.state.selected_topic !== null) ? this.state.selected_topic : 'None'
    const dp_namespace = namespace + '/' + data_product
    var image_topic = 'None'
    const { imageTopics } = this.props.ros
    var image_name = ''
    for (var i = 0; i < imageTopics.length; i++) {
      image_name = imageTopics[i].split('/').pop()
      if ((imageTopics[i].indexOf(dp_namespace) !== -1) && (image_name !== 'depth_map')) {
        image_topic = imageTopics[i]
        break
      }
    }
    return image_topic
  }

  renderImageViewer() {
    const image_topic = this.findImageTopic(this.state.data_product)
    const image_text = image_topic.split('/idx')[0].split('/').pop() + '-' + this.state.data_product

    return (
      <React.Fragment>
        <Columns>
          <Column equalWidth={false}>

            <NepiIFImageViewer
              image_topic={image_topic}
              title={image_text}
              data_product={this.state.data_product}
              hideQualitySelector={false}
              show_topic_selector={false}
              show_all_config_options={false}
            />

          </Column>
        </Columns>
      </React.Fragment>
    )
  }

  // The read-only half: values the node reports and the operator cannot set.
  renderStatus() {
    const { connected, enabled, selected_option, value } = this.state

    return (
      <React.Fragment>

        <Label title={"Connected"}>
          <BooleanIndicator value={connected} />
        </Label>

        <Label title={"Enabled"}>
          <BooleanIndicator value={enabled} />
        </Label>

        <Label title={"Selected Option"}>
          <Input disabled value={selected_option} />
        </Label>

        <Label title={"Value"}>
          <Input disabled value={value} />
        </Label>

      </React.Fragment>
    )
  }

  // One selector row per connect, selector only: device controls and data
  // are not rendered by this app. Nepi_IF_ConnectRBX draws its own title with
  // show_connect_header={true}. Nepi_IF_ConnectTargets takes the same props
  // the WPILib IF page passes it. The Obstacles row is built inline from
  // Label and Select, as on the WPILib IF page: there is no
  // Nepi_IF_ConnectObstacles.js, because ConnectObstaclesIF publishes no
  // ConnectIFStatus for one to bind to.
  renderConnections() {
    const appNamespace = this.getAppNamespace()

    if (appNamespace === null || appNamespace.indexOf('null') !== -1) {
      return null
    }

    const obstacles_namespaces = this.getObstaclesNamespaces()
    const obstacles_selected = this.getSelectedObstaclesNamespace()

    var obstacles_items = []
    obstacles_items.push(<Option value={OBSTACLES_NONE}>{OBSTACLES_NONE}</Option>)
    for (var i = 0; i < obstacles_namespaces.length; i++) {
      obstacles_items.push(
        <Option value={obstacles_namespaces[i]}>{obstacles_namespaces[i]}</Option>
      )
    }

    return (
      <React.Fragment>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        <Label title={"Connections"} />

        <NepiIFConnectRBX
          namespace={this.getConnectNamespace(RBX_CONNECT_NAME)}
          title={"Robot (RBX)"}
          show_connect_header={true}
          show_selector={true}
          show_controls={false}
          show_data={false}
          make_section={false}
        />

        <NepiIFConnectTargets
          namespace={this.getConnectNamespace(TARGETS_CONNECT_NAME)}
          title={"Targets"}
          show_selector={true}
          show_data={false}
          show_controls={false}
          shortened={true}
          make_section={false}
        />

        <Columns>
          <Column>

            <Label title={"Obstacles"}>
              <Select
                onChange={this.onObstaclesSelected}
                value={obstacles_selected}
              >
                {obstacles_items}
              </Select>
            </Label>

            <Label title={"Obstacles Connected"}>
              <BooleanIndicator value={this.state.obstacles_connected} />
            </Label>

          </Column>
        </Columns>

      </React.Fragment>
    )
  }

  // The adjustable half. title is passed as null because the Label above is
  // already this block's heading and the component's own default "CONTROLS"
  // title would print a second one under it. make_section={false} inlines the
  // set under the divider instead of wrapping it in its own Section.
  // allways_show_controls keeps the set open -- right when the controls ARE
  // the page. key={namespace} is required wherever the namespace can change at
  // runtime: without it React reuses the mounted component, which keeps its old
  // status subscription and leaves the previous values on screen.
  renderControls() {
    const controlsNamespace = this.getControlsNamespace()

    if (controlsNamespace === null || controlsNamespace.indexOf('null') !== -1) {
      return null
    }

    return (
      <React.Fragment>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        <Label title={"RobotStab Controls"} />

        <NepiIFControls
          key={controlsNamespace}
          namespace={controlsNamespace}
          title={null}
          make_section={false}
          allways_show_controls={true}
        />

      </React.Fragment>
    )
  }

  // NepiIFConfig stays mounted on the APP namespace, not the controls
  // namespace. A save there dumps the whole node parameter subtree, which
  // already includes the controls namespace, so one config box persists and
  // resets both.
  renderConfig() {
    const appNamespace = this.getAppNamespace()
    return (
      <React.Fragment>
        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />
        <NepiIFConfig
          namespace={appNamespace}
          title={"Nepi_IF_Config"}
        />
      </React.Fragment>
    )
  }

  // Right-hand 23% panel, top to bottom: status, connections, controls,
  // config.
  renderPanel() {
    return (
      <React.Fragment>
        {this.renderStatus()}
        {this.renderConnections()}
        {this.renderControls()}
        {this.renderConfig()}
      </React.Fragment>
    )
  }

  render() {
    const make_section = (this.props.make_section !== undefined) ? this.props.make_section : true
    const device_selected = (this.state.selected_topic !== null && this.state.selected_topic !== 'None')

    return (

      <Columns>
        <Column>

          <div style={{ display: 'flex' }}>

            <div style={{ width: "75%" }}>

              {this.renderSelection()}

              {(device_selected === true) ?
                this.renderImageViewer()
                : null}

            </div>

            <div style={{ width: '2%' }}>
              {}
            </div>

            <div style={{ width: "23%" }}>

              {(make_section === false) ?
                this.renderPanel()
                :
                <Section>
                  {this.renderPanel()}
                </Section>
              }

            </div>

          </div>

        </Column>
      </Columns>

    )
  }
}

export default NepiAppRobotStab
