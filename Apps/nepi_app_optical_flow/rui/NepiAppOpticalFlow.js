import React, { Component } from "react"
import { inject, observer } from "mobx-react"

import Section from "./Section"
import Label from "./Label"
import Select from "./Select"
import Input from "./Input"
import Button from "./Button"
import AsyncToggle from "./AsyncToggle"
import { SliderAdjustment } from "./AdjustmentWidgets"
import Styles from "./Styles"

const colors = Styles.vars.colors
const muted = {color: colors.grey1, fontSize: Styles.vars.fontSize.small}
const heading = {fontSize: Styles.vars.fontSize.regular, fontWeight: "bold",
  textTransform: "uppercase", marginBottom: Styles.vars.spacing.regular}

function DisplacementPlot({samples, fieldName, color}) {
  const width = 340, height = 155, left = 47, right = 10, top = 18, bottom = 25
  const values = samples.map(s => s[fieldName])
  const low = values.length ? Math.min(...values) : 0
  const high = values.length ? Math.max(...values) : 0
  const padding = Math.max((high - low) * 0.1, 0.1)
  const yMin = values.length ? low - padding : -1
  const yMax = values.length ? high + padding : 1
  const tMax = samples.length ? samples[samples.length - 1].t : 10
  const tMin = tMax - 10
  const plotW = width - left - right, plotH = height - top - bottom
  const xAt = t => left + ((t - tMin) / 10) * plotW
  const yAt = v => top + ((yMax - v) / (yMax - yMin)) * plotH
  const points = samples.map(s => `${xAt(s.t).toFixed(1)},${yAt(s[fieldName]).toFixed(1)}`).join(" ")
  const zero = yMin <= 0 && yMax >= 0 ? yAt(0) : null
  return <svg viewBox={`0 0 ${width} ${height}`}
    style={{width: "100%", display: "block", backgroundColor: colors.default_dark}}
    role="img" aria-label={`${fieldName.toUpperCase()} displacement over the past 10 seconds`}>
    <rect x={left} y={top} width={plotW} height={plotH}
      fill={colors.default_dark} stroke={colors.grey1} />
    {[0, 0.5, 1].map(f => <g key={f}>
      <line x1={left} x2={width-right} y1={top+f*plotH} y2={top+f*plotH}
        stroke={colors.grey2} />
      <text x={left-7} y={top+f*plotH+4} fill={colors.grey1}
        textAnchor="end" fontSize="11">{(yMax-f*(yMax-yMin)).toFixed(1)}</text>
    </g>)}
    {zero !== null && <line x1={left} x2={width-right} y1={zero} y2={zero}
      stroke={colors.grey1} strokeDasharray="4 4" />}
    {samples.length > 1 && <polyline points={points} fill="none" stroke={color}
      strokeWidth="2.5" strokeLinejoin="round" />}
    <text x={left} y={height-5} fill={colors.grey1} fontSize="11">−10 s</text>
    <text x={width-right} y={height-5} fill={colors.grey1}
      textAnchor="end" fontSize="11">now</text>
    <text x={5} y={10} fill={color} fontSize="13" fontWeight="bold">
      {fieldName.toUpperCase()} (px)
    </text>
  </svg>
}

@inject("ros")
@observer
class OpticalFlowApp extends Component {
  constructor(props) {
    super(props)
    this.state = {telemetry: {}, samples: [], k1Draft: null,
      pid: {p: "0", i: "0", d: "0"}}
    this.listener = null
    this.k1Timer = null
  }

  baseNamespace() {
    const {namespacePrefix, deviceId} = this.props.ros
    return namespacePrefix && deviceId ? `/${namespacePrefix}/${deviceId}/app_optical_flow` : null
  }

  componentDidMount() { this.subscribe() }
  componentDidUpdate() {
    if (this.subscribedNamespace !== this.baseNamespace()) this.subscribe()
  }
  componentWillUnmount() {
    if (this.listener) this.listener.unsubscribe()
    if (this.k1Timer) clearTimeout(this.k1Timer)
  }

  subscribe() {
    const namespace = this.baseNamespace()
    if (this.listener) this.listener.unsubscribe()
    if (this.k1Timer) clearTimeout(this.k1Timer)
    this.listener = null
    this.k1Timer = null
    this.subscribedNamespace = namespace
    if (namespace) this.listener = this.props.ros.setupStatusListener(
      `${namespace}/telemetry`, "std_msgs/String", message => {
        let telemetry
        try { telemetry = JSON.parse(message.data) } catch (_) { return }
        this.setState(previous => {
          let samples = telemetry.status === "Tracking" ? previous.samples : []
          if (typeof telemetry.t === "number" && typeof telemetry.x === "number" &&
              typeof telemetry.y === "number") {
            const last = samples[samples.length-1]
            if (!last || telemetry.t !== last.t) {
              samples = [...samples.filter(s => telemetry.t-s.t <= 10),
                {t: telemetry.t, x: telemetry.x, y: telemetry.y}]
            }
          }
          return {telemetry, samples,
            k1Draft: previous.k1Draft !== null &&
              Math.abs(previous.k1Draft - telemetry.k1) < 0.005 ? null : previous.k1Draft}
        })
      })
  }

  setK1(value) {
    const namespace = this.baseNamespace()
    if (!namespace) return
    this.setState({k1Draft: value})
    if (this.k1Timer) clearTimeout(this.k1Timer)
    this.k1Timer = setTimeout(() => {
      this.props.ros.sendFloatMsg(`${namespace}/set_k1`, value)
      this.k1Timer = null
    }, 150)
  }

  render() {
    const {telemetry, samples, pid, k1Draft} = this.state
    const namespace = this.baseNamespace()
    const cameras = telemetry.available_topics || (this.props.ros.imageTopics || [])
    const streamUrl = namespace ? `http://${window.location.hostname}:9091/stream?topic=${namespace}/annotated_image&type=mjpeg&quality=75` : ""
    const k1 = k1Draft !== null ? k1Draft :
      (typeof telemetry.k1 === "number" ? telemetry.k1 : -0.21)
    return <div style={{color: colors.white}}>
      <div style={{display: "grid", gridTemplateColumns: "minmax(0, 1.45fr) minmax(0, 1fr) minmax(240px, .8fr)",
        gap: Styles.vars.spacing.regular, alignItems: "start"}}>
        <Section title="Camera and motion path">
          <div style={{aspectRatio: "4 / 3", backgroundColor: colors.default_dark,
            border: `1px solid ${colors.grey1}`, display: "flex", alignItems: "center"}}>
            {namespace && <img src={streamUrl} alt="Live corrected camera with optical flow path"
              style={{width: "100%", maxHeight: "100%", objectFit: "contain"}} />}
          </div>
          <div style={{...muted, marginTop: Styles.vars.spacing.small}}>
            Green: camera motion history · Orange: current center
          </div>
          <Label title="Status">{telemetry.status || "Connecting"}</Label>
          <Label title="Camera source">
            <Select style={{maxWidth: "100%"}} value={telemetry.camera_topic || ""}
              disabled={!namespace} onChange={event => {
                this.props.ros.sendStringMsg(`${namespace}/set_image_topic`, event.target.value)
                this.setState({samples: []})
              }}>
              {!telemetry.camera_topic && <option value="">Waiting for cameras</option>}
              {telemetry.camera_topic && cameras.indexOf(telemetry.camera_topic) === -1 &&
                <option value={telemetry.camera_topic}>{telemetry.camera_topic} (unavailable)</option>}
              {cameras.map(topic => <option key={topic} value={topic}>{topic}</option>)}
            </Select>
          </Label>
        </Section>

        <Section title="Displacement">
          <DisplacementPlot samples={samples} fieldName="x" color={colors.blue} />
          <div style={{height: Styles.vars.spacing.regular}} />
          <DisplacementPlot samples={samples} fieldName="y" color={colors.orange} />
          <div style={{...muted, marginTop: Styles.vars.spacing.small}}>
            Camera axes; relative to the current reference
          </div>
        </Section>

        <Section title="Optical flow controls">
          <Label title="X displacement"><strong>{typeof telemetry.x === "number" ? `${telemetry.x.toFixed(1)} px` : "—"}</strong></Label>
          <Label title="Y displacement"><strong>{typeof telemetry.y === "number" ? `${telemetry.y.toFixed(1)} px` : "—"}</strong></Label>
          <Label title="Tracking rate"><strong>{telemetry.fps ? `${telemetry.fps} fps` : "—"}</strong></Label>
          <Label title="Features / inliers"><strong>{telemetry.features === undefined ? "—" : `${telemetry.features} / ${telemetry.inliers}`}</strong></Label>
          <Label title="Processing time"><strong>{telemetry.processing_ms === undefined ? "—" : `${telemetry.processing_ms} ms`}</strong></Label>
          <div style={{borderTop: `1px solid ${colors.grey1}`, marginTop: Styles.vars.spacing.regular,
            paddingTop: Styles.vars.spacing.regular}}>
            <SliderAdjustment title="Radial correction" topic={`${namespace}/set_k1`}
              msgType="std_msgs/Float32" adjustment={k1} min={-0.3} max={0.3}
              step={0.01} displayDecimals={2} unit="" noMargin
              disabled={!namespace} tooltip="0 disables correction; range −0.30 to +0.30"
              onSliderChangeOverride={value => this.setK1(value)} />
            <div style={{...muted, marginTop: Styles.vars.spacing.small}}>
              k1: −0.30 to +0.30 (0 = off)
            </div>
            <Label title="Mask ROV claw and cable">
              <AsyncToggle checked={!!telemetry.mask_foreground} disabled={!namespace}
                onClick={() => this.props.ros.sendBoolMsg(`${namespace}/set_mask_foreground`, !telemetry.mask_foreground)} />
            </Label>
            <div style={{marginTop: Styles.vars.spacing.regular}}>
              <Button buttonDownAction={null} buttonUpAction={() =>
                this.props.ros.sendBoolMsg(`${namespace}/reset_reference`, true)}
                disabled={!namespace}>Reset reference</Button>
            </div>
          </div>
          <div style={{borderTop: `1px solid ${colors.grey1}`, marginTop: Styles.vars.spacing.regular,
            paddingTop: Styles.vars.spacing.regular}}>
            <div style={heading}>Stabilization PID</div>
            <div style={muted}>Settings preview · actuator output inactive</div>
            {["p", "i", "d"].map(key => <Label key={key} title={`${key.toUpperCase()} gain`}>
              <Input type="number" step="0.01" value={pid[key]}
                onChange={event => this.setState({pid: {...pid, [key]: event.target.value}})}
                style={{width: "5em", backgroundColor: colors.grey0}} />
            </Label>)}
            <div style={{...muted, marginTop: Styles.vars.spacing.regular}}>
              Gains are local to this view until stabilization is connected.
            </div>
          </div>
        </Section>
      </div>
    </div>
  }
}

export default OpticalFlowApp
