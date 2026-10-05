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
 * NEPI APP TEMPLATE -- RUI PAGE
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
 * The page renders two halves:
 *   - what the operator ADJUSTS -> the shared Nepi_IF_Controls renderer, fed
 *     by the node's ControlsStatus. Do not hand-write a widget per value.
 *   - what the node REPORTS     -> read from this app's own status message.
 *
 * The namespace prop handed to Nepi_IF_Controls is the CONTROLS namespace
 * (<app>/controls), not the app namespace: the component appends '/status'
 * itself and its children append '/update_control'. Passing the app namespace
 * is the single most common way to get an empty control box -- a heading that
 * renders and then never fills in.
 */

import React, { Component } from "react"
import { observer, inject } from "mobx-react"

import Section from "./Section"
import { Columns, Column } from "./Columns"
import Label from "./Label"
import Input from "./Input"
import BooleanIndicator from "./BooleanIndicator"
import Styles from "./Styles"

import NepiIFControls from "./Nepi_IF_Controls"
import NepiIFConfig from "./Nepi_IF_Config"

// Must match CONTROLS_NAME in scripts/template_app_node.py. A ControlsIF is
// always a direct child of the node namespace, so the node and this page each
// derive the same name independently.
const CONTROLS_NAME = "controls"

@inject("ros")
@observer

// Template Application page
class NepiAppTemplate extends Component {

  constructor(props) {
    super(props)

    this.state = {
      // Must match APP_DICT.node_name in params/template_app_params.yaml.
      // apps_mgr launches the node under the yaml's node_name, so the yaml --
      // not DEFAULT_NODE_NAME in the script -- is what this has to agree with.
      appName: "app_template",
      appNamespace: null,

      // Read-only status fields — mirror NepiAppTemplateStatus.msg
      enabled: false,
      selected_option: "None",
      value: 0.0,

      statusListener: null,
      connected: false,
    }

    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.getControlsNamespace = this.getControlsNamespace.bind(this)
    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)
    this.renderStatus = this.renderStatus.bind(this)
    this.renderControls = this.renderControls.bind(this)
    this.renderConfig = this.renderConfig.bind(this)
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
      "nepi_app_template/NepiAppTemplateStatus",
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

        <Label title={"Template Controls"} />

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

  render() {
    const make_section = (this.props.make_section !== undefined) ? this.props.make_section : true

    if (make_section === false) {
      return (
        <Columns>
          <Column>
            {this.renderStatus()}
            {this.renderControls()}
            {this.renderConfig()}
          </Column>
        </Columns>
      )
    } else {
      return (
        <Section>
          {this.renderStatus()}
          {this.renderControls()}
          {this.renderConfig()}
        </Section>
      )
    }
  }
}

export default NepiAppTemplate
