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
 *   - left 75%:  the image viewer, driven by the IDX connect's selection.
 *   - right 23%: what the node REPORTS (this app's own status message), the
 *     device connect selector rows, then what the operator ADJUSTS (the shared
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
 * discovery, the persisted selection, ConnectIFStatus and select_topic. This
 * page selects nothing itself.
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
import NepiIFConnectIDX from "./Nepi_IF_ConnectIDX"
import NepiIFConnectLSX from "./Nepi_IF_ConnectLSX"
import NepiIFConnectMotor from "./Nepi_IF_ConnectMotor"
import NepiIFConnectNPX from "./Nepi_IF_ConnectNPX"

import NepiIFControls from "./Nepi_IF_Controls"
import NepiIFConfig from "./Nepi_IF_Config"

// Must match CONTROLS_NAME in scripts/robot_stab_app_node.py. A ControlsIF is
// always a direct child of the node namespace, so the node and this page each
// derive the same name independently.
const CONTROLS_NAME = "controls"

// Must match CONNECT_NAME in each nepi_api/connect_device_if_<type>.py. The
// node constructs every connect with its default name. The servo connect
// (svx_connect) has no row yet -- see renderConnections().
const IDX_CONNECT_NAME = "idx_connect"
const LSX_CONNECT_NAME = "lsx_connect"
const MOTOR_CONNECT_NAME = "motor_connect"
const NPX_CONNECT_NAME = "npx_connect"

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
      // *_connected fields are not repeated here: each connect row below
      // renders its own Connected indicator.
      enabled: false,
      selected_option: "None",
      value: 0.0,

      statusListener: null,
      connected: false,

      // IDX connect namespace (<app>/idx_connect) the second listener is
      // pointed at
      idxConnectNamespace: null,
      idxConnectStatusListener: null,

      // Selected camera topic (<device>/idx), sourced from ConnectIFStatus
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
    this.updateIdxConnectStatusListener = this.updateIdxConnectStatusListener.bind(this)
    this.idxConnectStatusListener = this.idxConnectStatusListener.bind(this)

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

  // Second listener, on the IDX connect's own status topic
  // (<app>/idx_connect/status, ConnectIFStatus). The page reads it only to
  // learn which camera is selected, so the image viewer can follow it.
  updateIdxConnectStatusListener() {
    const namespace = this.getConnectNamespace(IDX_CONNECT_NAME)
    if (this.state.idxConnectStatusListener != null) {
      this.state.idxConnectStatusListener.unsubscribe()
      this.setState({ idxConnectStatusListener: null })
    }
    if (namespace != null && namespace !== 'None') {
      var idxConnectStatusListener = this.props.ros.setupStatusListener(
        namespace + '/status',
        "nepi_interfaces/ConnectIFStatus",
        this.idxConnectStatusListener
      )
      this.setState({ idxConnectStatusListener: idxConnectStatusListener })
    }
    this.setState({ idxConnectNamespace: namespace })
  }

  // Clears the data product selection whenever the selected camera changes so
  // the image viewer re-resolves.
  idxConnectStatusListener(message) {
    if (message.selected_topic !== this.state.selected_topic) {
      this.setState({
        selected_topic: message.selected_topic,
        data_topic: 'None',
        data_product: 'None'
      })
    }
  }

  componentDidMount() {
    const namespace = this.getAppNamespace()
    if (namespace !== null) {
      this.updateStatusListener(namespace)
    }
    this.updateIdxConnectStatusListener()
  }

  componentDidUpdate(prevProps, prevState) {
    const namespace = this.getAppNamespace()
    const namespace_updated = (this.state.appNamespace !== namespace && namespace !== null)
    if (namespace_updated) {
      if (namespace.indexOf('null') === -1) {
        this.updateStatusListener(namespace)
      }
    }
    // Re-point the IDX connect listener when its namespace resolves or changes.
    const idxConnectNamespace = this.getConnectNamespace(IDX_CONNECT_NAME)
    if (idxConnectNamespace !== this.state.idxConnectNamespace) {
      this.updateIdxConnectStatusListener()
    }
  }

  componentWillUnmount() {
    if (this.state.statusListener) {
      this.state.statusListener.unsubscribe()
    }
    if (this.state.idxConnectStatusListener) {
      this.state.idxConnectStatusListener.unsubscribe()
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

  // Data product selection section. Local to this page and drives the image
  // viewer; the camera selection itself belongs to the IDX connect row.
  renderSelection() {
    const device_selected = (this.state.selected_topic !== null && this.state.selected_topic !== 'None')

    if (device_selected === false) {
      return (
        <Columns>
          <Column>

          </Column>
        </Columns>
      )
    }

    return (
      <Section title={"Selection"}>

        {this.renderDataProductSelector()}

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

  // One selector row per device connect, selector only: device controls and
  // data are not rendered by this app. With make_section={false} every row's
  // own label is just "Device", so each row needs a title above it.
  // Nepi_IF_ConnectIDX and Nepi_IF_ConnectMotor draw that title themselves
  // with show_connect_header={true}. Nepi_IF_ConnectLSX and Nepi_IF_ConnectNPX
  // have no header option and read title only for their own Section, so this
  // page draws a matching bold Label above those two instead.
  renderConnections() {
    const appNamespace = this.getAppNamespace()

    if (appNamespace === null || appNamespace.indexOf('null') !== -1) {
      return null
    }

    return (
      <React.Fragment>

        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }} />

        <Label title={"Device Connections"} />

        {/* SERVO ROW -- PENDING: Nepi_IF_ConnectSVX.js does not exist yet (a missing import breaks the whole RUI build).
            Mount from NepiAppSVXConnect.js; on uncommenting, use this.getConnectNamespace("svx_connect") and the rows' selector-only props.

            import NepiIFConnectSVX from "./Nepi_IF_ConnectSVX"
            ...
            <NepiIFConnectSVX namespace={connectNamespace} title={"SVX Connect"}
                              show_selector={true} show_data={true}
                              make_section={make_section} />
        */}

        <NepiIFConnectIDX
          namespace={this.getConnectNamespace(IDX_CONNECT_NAME)}
          title={"Camera (IDX)"}
          show_connect_header={true}
          show_selector={true}
          show_controls={false}
          show_data={false}
          make_section={false}
        />

        <Label title={"Light (LSX)"} labelStyle={{fontWeight: 'bold'}} />
        <NepiIFConnectLSX
          namespace={this.getConnectNamespace(LSX_CONNECT_NAME)}
          show_selector={true}
          show_controls={false}
          show_data={false}
          make_section={false}
        />

        <NepiIFConnectMotor
          namespace={this.getConnectNamespace(MOTOR_CONNECT_NAME)}
          title={"Motor"}
          show_connect_header={true}
          show_selector={true}
          show_controls={false}
          show_data={false}
          make_section={false}
        />

        <Label title={"NavPose (NPX)"} labelStyle={{fontWeight: 'bold'}} />
        <NepiIFConnectNPX
          namespace={this.getConnectNamespace(NPX_CONNECT_NAME)}
          show_selector={true}
          show_controls={false}
          show_data={false}
          make_section={false}
        />

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

  // Right-hand 23% panel, top to bottom: status, device connections, controls,
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
