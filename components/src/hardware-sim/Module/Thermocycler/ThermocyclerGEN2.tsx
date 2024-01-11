import * as React from 'react'

import { LEGACY_COLORS } from '../../../ui-style-constants'
import { COLORS } from '../../../helix-design-system'

interface ThermocyclerGEN2Props {
  lidMotorState: 'open' | 'closed'
  ledLightColor: string
}

export function ThermocyclerGEN2(props: ThermocyclerGEN2Props): JSX.Element {
  return (
    <g id="thermocyclerGEN2">
      <g id="buttonDarkFill">
        <path
          d="M154.3,26.6c0-2.2-1.8-4.1-4.1-4.1s-4.1,1.8-4.1,4.1c0,2.2,1.8,4.1,4.1,4.1C152.5,30.7,154.3,28.9,154.3,26.6 z"
          style={{ fill: '#E6E6E6' }}
        ></path>
      </g>
      <g id="statusLight">
        <path d="M130.4,23.2H41.6c-1.9,0-3.4,1.5-3.4,3.4s1.5,3.4,3.4,3.4h88.8c1.9,0,3.4-1.5,3.4-3.4C133.8,24.7,132.3,23.2,130.4,23.2z M41.6,29.4c-1.5,0-2.7-1.2-2.7-2.7s1.2-2.7,2.7-2.7h88.8c1.5,0,2.7,1.2,2.7,2.7s-1.2,2.7-2.7,2.7H41.6z"></path>
      </g>
      <g id="button">
        <path d="M150.3,17.4c-5.1,0-9.2,4.1-9.2,9.2s4.1,9.2,9.2,9.2c5.1,0,9.2-4.1,9.2-9.2S155.3,17.4,150.3,17.4z M150.3,35.1 c-4.7,0-8.5-3.8-8.5-8.5s3.8-8.5,8.5-8.5c4.7,0,8.5,3.8,8.5,8.5S154.9,35.1,150.3,35.1z M150.3,20.2c-3.5,0-6.4,2.9-6.4,6.4 s2.9,6.4,6.4,6.4c3.5,0,6.4-2.9,6.4-6.4S153.8,20.2,150.3,20.2z M150.3,32.4c-3.2,0-5.7-2.6-5.7-5.7s2.6-5.7,5.7-5.7 c3.2,0,5.7,2.6,5.7,5.7S153.4,32.4,150.3,32.4z M150.3,22.2c-2.4,0-4.4,2-4.4,4.4s2,4.4,4.4,4.4s4.4-2,4.4-4.4 S152.7,22.2,150.3,22.2z M150.3,30.3c-2,0-3.7-1.7-3.7-3.7s1.7-3.7,3.7-3.7c2,0,3.7,1.7,3.7,3.7S152.3,30.3,150.3,30.3z"></path>
      </g>
      <ClosedThermocyclerGEN2Layers ledLightColor={props.ledLightColor} />
      {props.lidMotorState === 'open' ? <OpenThermocyclerGEN2Layers /> : null}
    </g>
  )
}

function ClosedThermocyclerGEN2Layers(props: {
  ledLightColor: string
}): JSX.Element {
  return (
    <g id="closed_thermocycler_gen2">
      <path
        style={{ fill: COLORS.white }}
        d="M145.2,232.7V186c0-4-2.7-7.2-6-7.2H32.5c-3.3,0-5.7,3.2-5.7,7.2v46.7H0.4V0.4h171.2v232.3H145.2z"
      ></path>
      <path
        style={{ fill: '#E6E6E6' }}
        d="M139.1,178.8h-106c-3.3,0-6,2.7-6,6v47.9h118v-47.9C145.1,181.5,142.4,178.8,139.1,178.8z"
      ></path>
      <rect
        x="4.2"
        y="0.4"
        style={{ fill: '#E6E6E6' }}
        width="163.4"
        height="38.3"
      ></rect>
      <path
        style={{ fill: COLORS.white }}
        d="M141,191.8H31.1c-1.1,0-2,0.9-2,2v49.2c0,1.1,0.9,2,2,2H141c1.1,0,2-0.9,2-2v-49.2 C143,192.7,142.1,191.8,141,191.8z"
      ></path>
      <rect
        x="32.7"
        y="195.4"
        style={{ fill: '#E6E6E6' }}
        width="106.8"
        height="49.5"
      ></rect>
      <path
        style={{ fill: COLORS.white }}
        d="M130.8,18.5H41.2c-1.6,0-3,1.3-3,3l0,0c0,1.7,1.3,3,3,3h89.6c1.6,0,3-1.3,3-3l0,0 C133.8,19.9,132.5,18.5,130.8,18.5z"
      ></path>
      <path
        fill={props.ledLightColor}
        stroke={LEGACY_COLORS.black}
        strokeWidth={0.5}
        d="M130.8,18.3H41.2c-1.7,0-3.2,1.4-3.2,3.2c0,1.7,1.4,3.2,3.2,3.2h89.6c1.7,0,3.2-1.4,3.2-3.2 C133.9,19.8,132.5,18.3,130.8,18.3z"
      ></path>
      <path d="M4.4,0.2c-0.1,0-0.2,0.1-0.2,0.2v232.3c0,0.1,0.1,0.2,0.2,0.2c0.1,0,0.2-0.1,0.2-0.2V0.4C4.6,0.3,4.5,0.2,4.4,0.2z"></path>
      <path d="M171.6,38.3H0.4c-0.2,0-0.4,0.2-0.4,0.4S0.2,39,0.4,39h171.2c0.2,0,0.4-0.2,0.4-0.4S171.8,38.3,171.6,38.3z"></path>
      <path d="M167.6,0.2c-0.1,0-0.2,0.1-0.2,0.2v232.4c0,0.1,0.1,0.2,0.2,0.2c0.1,0,0.2-0.1,0.2-0.2V0.4 C167.8,0.3,167.7,0.2,167.6,0.2z"></path>
      <path d="M139.5,242.3c-0.1,0-0.1,0.1-0.1,0.1v2.5c0,0.1,0.1,0.1,0.1,0.1s0.1-0.1,0.1-0.1v-2.5 C139.6,242.4,139.5,242.3,139.5,242.3z"></path>
      <path d="M139.1,195.1H139c-0.1,0-0.1,0.1-0.1,0.1s0.1,0.1,0.1,0.1h0.1l0.2,0.1l0.1,0.1l0,0.1c0,0.1,0.1,0.1,0.1,0.1 s0.1-0.1,0.1-0.1v-0.1c0,0,0,0,0,0l-0.1-0.2l-0.1-0.1c0,0,0,0,0,0L139.1,195.1C139.1,195.1,139.1,195.1,139.1,195.1z"></path>
      <path d="M33.1,195.1H33c0,0,0,0,0,0l-0.1,0c0,0,0,0,0,0l-0.1,0.1l-0.1,0c0,0,0,0,0,0l-0.1,0.2l0,0.1c0,0,0,0,0,0v0.1 c0,0.1,0.1,0.1,0.1,0.1c0.1,0,0.1-0.1,0.1-0.1v0l0.1-0.1l0-0.1l0,0c0,0,0,0,0,0l0,0l0.1,0h0.1c0.1,0,0.1-0.1,0.1-0.1 C33.2,195.2,33.1,195.1,33.1,195.1z"></path>
      <path d="M32.6,242.3c-0.1,0-0.1,0.1-0.1,0.1v2.5c0,0.1,0.1,0.1,0.1,0.1c0.1,0,0.1-0.1,0.1-0.1v-2.5 C32.7,242.4,32.6,242.3,32.6,242.3z"></path>
      <path d="M0.4,0C0.2,0,0.1,0.2,0.1,0.4v232.3c0,0.2,0.2,0.4,0.4,0.4h28.7c0.2,0,0.4-0.2,0.4-0.4c0-0.2-0.2-0.4-0.4-0.4H0.8V0.7 h170.5v231.7H143c-0.2,0-0.4,0.2-0.4,0.4s0.2,0.4,0.4,0.4h28.7c0.2,0,0.4-0.2,0.4-0.4V0.4c0-0.2-0.2-0.4-0.4-0.4H0.5 C0.5,0,0.4,0,0.4,0z"></path>
      <path d="M171.6,4.2H0.4c-0.1,0-0.2,0.1-0.2,0.2c0,0.1,0.1,0.2,0.2,0.2h171.2c0.1,0,0.2-0.1,0.2-0.2 C171.8,4.3,171.7,4.2,171.6,4.2z"></path>
      <path d="M150.6,12.4c-5,0-9.1,4.1-9.1,9.1s4.1,9.1,9.1,9.1s9.1-4.1,9.1-9.1S155.6,12.4,150.6,12.4z M150.6,30.2 c-4.8,0-8.7-3.9-8.7-8.7s3.9-8.7,8.7-8.7s8.7,3.9,8.7,8.7S155.4,30.2,150.6,30.2z"></path>
      <ellipse
        style={{ fill: COLORS.white }}
        cx="150.6"
        cy="21.5"
        rx="5.8"
        ry="5.8"
      ></ellipse>
      <path d="M150.6,15.5c-3.3,0-6,2.7-6,6s2.7,6,6,6s6-2.7,6-6S153.9,15.5,150.6,15.5z M150.6,27.2c-3.1,0-5.7-2.5-5.7-5.7 s2.5-5.7,5.7-5.7s5.7,2.5,5.7,5.7S153.7,27.2,150.6,27.2z"></path>
      <ellipse
        style={{ fill: '#E6E6E6' }}
        cx="150.6"
        cy="21.5"
        rx="3.8"
        ry="3.8"
      ></ellipse>
      <path d="M150.6,17.6c-2.2,0-3.9,1.8-3.9,4s1.8,4,3.9,4c2.2,0,3.9-1.8,3.9-4C154.5,19.3,152.8,17.6,150.6,17.6z M150.6,25.1 c-2,0-3.6-1.6-3.6-3.6s1.6-3.6,3.6-3.6s3.6,1.6,3.6,3.6S152.6,25.1,150.6,25.1z"></path>
      <path d="M141,191.4H31.1c-1.3,0-2.3,1.1-2.3,2.3v51.2c0,0.2,0.2,0.4,0.4,0.4H143c0.2,0,0.4-0.2,0.4-0.4v-51.2 C143.3,192.5,142.3,191.4,141,191.4z M29.5,244.6v-50.8c0-0.9,0.7-1.6,1.6-1.6H141c0.9,0,1.6,0.7,1.6,1.6v50.8H29.5z"></path>
      <path d="M139,195.1H33.1c-0.4,0-0.7,0.3-0.7,0.7v49.2c0,0.1,0.1,0.2,0.2,0.2h106.9c0.1,0,0.2-0.1,0.2-0.2v-49.2 C139.7,195.4,139.4,195.1,139,195.1z M32.8,244.8v-49c0-0.2,0.1-0.3,0.3-0.3H139c0.2,0,0.3,0.1,0.3,0.3v49H32.8z"></path>
      <path d="M139.1,178.4H32.7c-3.5,0-6.3,2.8-6.3,6.3V233h0.7v-48.2c0-3.1,2.5-5.6,5.6-5.6h106.4c3.1,0,5.6,2.5,5.6,5.6v47.7h0.7 v-47.7C145.4,181.3,142.6,178.4,139.1,178.4z"></path>
      <path d="M51.7,79.2c-0.1,0-0.1,0.1-0.1,0.1v99.4c0,0.1,0.1,0.1,0.1,0.1c0.1,0,0.1-0.1,0.1-0.1V79.3 C51.8,79.2,51.8,79.2,51.7,79.2z"></path>
      <path d="M120.4,79.1c-0.1,0-0.2,0.1-0.2,0.2v99.4c0,0.1,0.1,0.2,0.2,0.2c0.1,0,0.2-0.1,0.2-0.2V79.3 C120.6,79.2,120.5,79.1,120.4,79.1z"></path>
      <path d="M120.4,79.1H51.6c-0.1,0-0.2,0.1-0.2,0.2c0,0.1,0.1,0.2,0.2,0.2h68.7c0.1,0,0.2-0.1,0.2-0.2 C120.6,79.2,120.5,79.1,120.4,79.1z"></path>
    </g>
  )
}

function OpenThermocyclerGEN2Layers(): JSX.Element {
  return (
    <g id="open_thermocycler_gen2">
      <g id="shading">
        <path
          style={{ fill: COLORS.white }}
          d="M51.7,247.3c0,2.4,0,9.8,0,11.1c0,1.4,0,2.8,1.1,3.9c1.2,1.2,2.6,1.2,4.1,1.2c1.6,0,58,0,59.1,0 c1.3,0,2.5-0.3,3.3-1.3c1.3-1.4,1-3.3,1-5c0-1.8,0-8.4,0-9.9"
        ></path>
        <rect
          x="55.6"
          y="246.9"
          style={{ fill: '#E6E6E6' }}
          width="60.7"
          height="12.7"
        ></rect>
        <path
          style={{ fill: COLORS.white }}
          d="M171.6,38.7H0.4v206.3c0,1.1,0.9,2,2,2h167.3c1.1,0,2-0.9,2-2L171.6,38.7L171.6,38.7z"
        ></path>
        <path d="M171.6,38.3H0.4c-0.2,0-0.4,0.2-0.4,0.4v206.3c0,1.3,1.1,2.3,2.3,2.3h167.3c1.3,0,2.3-1.1,2.3-2.3V38.7 C172,38.5,171.8,38.3,171.6,38.3z M0.7,39h170.6v205.9c0,0.9-0.7,1.6-1.6,1.6H2.3c-0.9,0-1.6-0.7-1.6-1.6 C0.7,244.9,0.7,39,0.7,39z"></path>
      </g>
      <g id="base_grey">
        <rect
          x="4.4"
          y="39.1"
          style={{ fill: '#E6E6E6' }}
          width="163.4"
          height="203.4"
        ></rect>
        <path
          style={{ fill: COLORS.white }}
          d="M19.3,70.9v-4.4c0-1.2,1-2.2,2.2-2.2h129c1.2,0,2.2,1,2.2,2.2v4.4h-5.9c-0.6,0-1,0.4-1,1v2.5 c0,0.6,0.4,1,1,1h5.9v65.3h-5.9c-0.6,0-1,0.4-1,1v2.5c0,0.6,0.4,1,1,1h5.9V156c0,1.2-1,2.2-2.2,2.2h-129l0,0 c-1.2,0-2.2-1-2.2-2.2v0v-10.8h5.9c0.6,0,1-0.4,1-1v-2.5c0-0.6-0.4-1-1-1h-5.9V75.4h5.9c0.6,0,1-0.4,1-1v-2.5c0-0.6-0.4-1-1-1 L19.3,70.9"
        ></path>
        <path
          style={{ fill: COLORS.white }}
          d="M143,203v-9.2c0-1.1-0.9-2-2-2H31.1c-1.1,0-2,0.9-2,2v9.2"
        ></path>
        <polyline
          style={{ fill: '#E6E6E6' }}
          points="139.5,203.1 139.5,195.4 32.8,195.4 32.8,203.3 			"
        ></polyline>
      </g>
      <g id="stroke">
        <path d="M116.5,246.7h-0.4v12.7H55.8v-12.7h-0.4v12.9c0,0.1,0.1,0.2,0.2,0.2h60.7c0.1,0,0.2-0.1,0.2-0.2V246.7z"></path>
        <path d="M120.4,246.6c-0.2,0-0.4,0.2-0.4,0.4v10.2c0,0.3,0,0.5,0,0.8c0,1.5,0.1,3-0.9,4.2c-0.7,0.8-1.7,1.2-3.1,1.2H56.9 c-1.4,0-2.8,0-3.8-1.1c-1-1.1-1-2.5-1-3.8v-11.4c0-0.2-0.2-0.4-0.4-0.4s-0.4,0.2-0.4,0.4v11.4c0,1.4,0,3,1.2,4.3 c1.3,1.3,2.8,1.3,4.4,1.3H116c1.5,0,2.7-0.5,3.6-1.5c1.2-1.3,1.1-3.1,1.1-4.6c0-0.3,0-0.5,0-0.8v-10.2 C120.7,246.7,120.6,246.6,120.4,246.6z"></path>
      </g>
      <g id="row_A_block_wells">
        <path d="M45.7,139.7c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S47.3,139.7,45.7,139.7z M45.7,145.3c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S47.2,145.3,45.7,145.3z"></path>
        <path d="M54.7,139.7c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S56.3,139.7,54.7,139.7z M54.7,145.3c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S56.1,145.3,54.7,145.3z"></path>
        <path d="M63.6,139.7c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S65.3,139.7,63.6,139.7z M63.6,145.3c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S65.1,145.3,63.6,145.3z"></path>
        <path d="M72.6,139.7c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S74.2,139.7,72.6,139.7z M72.6,145.3c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S74,145.3,72.6,145.3z"></path>
        <path d="M81.6,139.7c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S83.2,139.7,81.6,139.7z M81.6,145.3c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S83,145.3,81.6,145.3z"></path>
        <path d="M90.5,139.7c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S92.1,139.7,90.5,139.7z M90.5,145.3c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S91.9,145.3,90.5,145.3z"></path>
        <path d="M99.5,139.7c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S101.1,139.7,99.5,139.7z M99.5,145.3c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S100.9,145.3,99.5,145.3z"></path>
        <path d="M108.4,139.7c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S110.1,139.7,108.4,139.7z M108.4,145.3c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S109.9,145.3,108.4,145.3z"></path>
        <path d="M117.4,139.7c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S119,139.7,117.4,139.7z M117.4,145.3 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S118.8,145.3,117.4,145.3z"></path>
        <path d="M126.3,139.7c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S128,139.7,126.3,139.7z M126.3,145.3c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S127.8,145.3,126.3,145.3z"></path>
        <path d="M135.3,139.7c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S137,139.7,135.3,139.7z M135.3,145.3c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S136.8,145.3,135.3,145.3z"></path>
        <path d="M36.7,139.7c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S38.4,139.7,36.7,139.7z M36.7,145.3c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S38.2,145.3,36.7,145.3z"></path>
      </g>

      <g id="row_B_block_wells">
        <path d="M45.7,130.7c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S47.3,130.7,45.7,130.7z M45.7,136.3c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S47.2,136.3,45.7,136.3z"></path>
        <path d="M54.7,130.7c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S56.3,130.7,54.7,130.7z M54.7,136.3c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S56.1,136.3,54.7,136.3z"></path>
        <path d="M63.6,130.7c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S65.3,130.7,63.6,130.7z M63.6,136.3c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S65.1,136.3,63.6,136.3z"></path>
        <path d="M72.6,130.7c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S74.2,130.7,72.6,130.7z M72.6,136.3c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S74,136.3,72.6,136.3z"></path>
        <path d="M81.6,130.7c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S83.2,130.7,81.6,130.7z M81.6,136.3c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S83,136.3,81.6,136.3z"></path>
        <path d="M90.5,130.7c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S92.1,130.7,90.5,130.7z M90.5,136.3c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S91.9,136.3,90.5,136.3z"></path>
        <path d="M99.5,130.7c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S101.1,130.7,99.5,130.7z M99.5,136.3c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S100.9,136.3,99.5,136.3z"></path>
        <path d="M108.4,130.7c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S110.1,130.7,108.4,130.7z M108.4,136.3c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S109.9,136.3,108.4,136.3z"></path>
        <path d="M117.4,130.7c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S119,130.7,117.4,130.7z M117.4,136.3 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S118.8,136.3,117.4,136.3z"></path>
        <path d="M126.3,130.7c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S128,130.7,126.3,130.7z M126.3,136.3c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S127.8,136.3,126.3,136.3z"></path>
        <path d="M135.3,130.7c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S137,130.7,135.3,130.7z M135.3,136.3c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S136.8,136.3,135.3,136.3z"></path>
        <path d="M36.7,130.7c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S38.4,130.7,36.7,130.7z M36.7,136.3c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S38.2,136.3,36.7,136.3z"></path>
      </g>

      <g id="row_C_block_wells">
        <path d="M45.7,121.8c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3s3-1.3,3-3C48.7,123.1,47.3,121.8,45.7,121.8z M45.7,127.4 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S47.2,127.4,45.7,127.4z"></path>
        <path d="M54.7,121.8c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3c1.6,0,3-1.3,3-3C57.7,123.1,56.3,121.8,54.7,121.8z M54.7,127.4 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S56.1,127.4,54.7,127.4z"></path>
        <path d="M63.6,121.8c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3c1.6,0,3-1.3,3-3C66.6,123.1,65.3,121.8,63.6,121.8z M63.6,127.4 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S65.1,127.4,63.6,127.4z"></path>
        <path d="M72.6,121.8c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3c1.6,0,3-1.3,3-3C75.6,123.1,74.2,121.8,72.6,121.8z M72.6,127.4 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S74,127.4,72.6,127.4z"></path>
        <path d="M81.6,121.8c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3s3-1.3,3-3C84.5,123.1,83.2,121.8,81.6,121.8z M81.6,127.4 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S83,127.4,81.6,127.4z"></path>
        <path d="M90.5,121.8c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3s3-1.3,3-3C93.5,123.1,92.1,121.8,90.5,121.8z M90.5,127.4 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S91.9,127.4,90.5,127.4z"></path>
        <path d="M99.5,121.8c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3s3-1.3,3-3C102.4,123.1,101.1,121.8,99.5,121.8z M99.5,127.4 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S100.9,127.4,99.5,127.4z"></path>
        <path d="M108.4,121.8c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3s3-1.3,3-3C111.4,123.1,110.1,121.8,108.4,121.8z M108.4,127.4 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S109.9,127.4,108.4,127.4z"></path>
        <path d="M117.4,121.8c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3c1.6,0,3-1.3,3-3C120.3,123.1,119,121.8,117.4,121.8z M117.4,127.4 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S118.8,127.4,117.4,127.4z"></path>
        <path d="M126.3,121.8c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3s3-1.3,3-3C129.3,123.1,128,121.8,126.3,121.8z M126.3,127.4 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S127.8,127.4,126.3,127.4z"></path>
        <path d="M135.3,121.8c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3s3-1.3,3-3C138.3,123.1,137,121.8,135.3,121.8z M135.3,127.4 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S136.8,127.4,135.3,127.4z"></path>
        <path d="M36.7,121.8c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3s3-1.3,3-3C39.7,123.1,38.4,121.8,36.7,121.8z M36.7,127.4 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S38.2,127.4,36.7,127.4z"></path>
      </g>

      <g id="row_D_block_wells">
        <path d="M45.7,112.8c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S47.3,112.8,45.7,112.8z M45.7,118.4c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6C48.3,117.2,47.2,118.4,45.7,118.4z"></path>
        <path d="M54.7,112.8c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S56.3,112.8,54.7,112.8z M54.7,118.4c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6C57.3,117.2,56.1,118.4,54.7,118.4z"></path>
        <path d="M63.6,112.8c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S65.3,112.8,63.6,112.8z M63.6,118.4c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6C66.3,117.2,65.1,118.4,63.6,118.4z"></path>
        <path d="M72.6,112.8c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S74.2,112.8,72.6,112.8z M72.6,118.4c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6C75.2,117.2,74,118.4,72.6,118.4z"></path>
        <path d="M81.6,112.8c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S83.2,112.8,81.6,112.8z M81.6,118.4c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6C84.2,117.2,83,118.4,81.6,118.4z"></path>
        <path d="M90.5,112.8c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S92.1,112.8,90.5,112.8z M90.5,118.4c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6C93.1,117.2,91.9,118.4,90.5,118.4z"></path>
        <path d="M99.5,112.8c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S101.1,112.8,99.5,112.8z M99.5,118.4c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6C102.1,117.2,100.9,118.4,99.5,118.4z"></path>
        <path d="M108.4,112.8c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S110.1,112.8,108.4,112.8z M108.4,118.4c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6C111,117.2,109.9,118.4,108.4,118.4z"></path>
        <path d="M117.4,112.8c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S119,112.8,117.4,112.8z M117.4,118.4 c-1.4,0-2.6-1.2-2.6-2.6c0-1.4,1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6C120,117.2,118.8,118.4,117.4,118.4z"></path>
        <path d="M126.3,112.8c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S128,112.8,126.3,112.8z M126.3,118.4c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6C129,117.2,127.8,118.4,126.3,118.4z"></path>
        <path d="M135.3,112.8c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S137,112.8,135.3,112.8z M135.3,118.4c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6C137.9,117.2,136.8,118.4,135.3,118.4z"></path>
        <path d="M36.7,112.8c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S38.4,112.8,36.7,112.8z M36.7,118.4c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6C39.4,117.2,38.2,118.4,36.7,118.4z"></path>
      </g>

      <g id="row_E_block_wells">
        <path d="M45.7,103.8c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S47.3,103.8,45.7,103.8z M45.7,109.4c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6C48.3,108.2,47.2,109.4,45.7,109.4z"></path>
        <path d="M54.7,103.8c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S56.3,103.8,54.7,103.8z M54.7,109.4c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6C57.3,108.2,56.1,109.4,54.7,109.4z"></path>
        <path d="M63.6,103.8c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S65.3,103.8,63.6,103.8z M63.6,109.4c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6C66.3,108.2,65.1,109.4,63.6,109.4z"></path>
        <path d="M72.6,103.8c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S74.2,103.8,72.6,103.8z M72.6,109.4c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6C75.2,108.2,74,109.4,72.6,109.4z"></path>
        <path d="M81.6,103.8c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S83.2,103.8,81.6,103.8z M81.6,109.4c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6C84.2,108.2,83,109.4,81.6,109.4z"></path>
        <path d="M90.5,103.8c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S92.1,103.8,90.5,103.8z M90.5,109.4c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6C93.1,108.2,91.9,109.4,90.5,109.4z"></path>
        <path d="M99.5,103.8c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S101.1,103.8,99.5,103.8z M99.5,109.4c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6C102.1,108.2,100.9,109.4,99.5,109.4z"></path>
        <path d="M108.4,103.8c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S110.1,103.8,108.4,103.8z M108.4,109.4c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6C111,108.2,109.9,109.4,108.4,109.4z"></path>
        <path d="M117.4,103.8c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S119,103.8,117.4,103.8z M117.4,109.4 c-1.4,0-2.6-1.2-2.6-2.6c0-1.4,1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6C120,108.2,118.8,109.4,117.4,109.4z"></path>
        <path d="M126.3,103.8c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S128,103.8,126.3,103.8z M126.3,109.4c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6C129,108.2,127.8,109.4,126.3,109.4z"></path>
        <path d="M135.3,103.8c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S137,103.8,135.3,103.8z M135.3,109.4c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6C137.9,108.2,136.8,109.4,135.3,109.4z"></path>
        <path d="M36.7,103.8c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S38.4,103.8,36.7,103.8z M36.7,109.4c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6C39.4,108.2,38.2,109.4,36.7,109.4z"></path>
      </g>

      <g id="row_F_block_wells">
        <path d="M45.7,94.9c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S47.3,94.9,45.7,94.9z M45.7,100.5c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6C48.3,99.3,47.2,100.5,45.7,100.5z"></path>
        <path d="M54.7,94.9c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S56.3,94.9,54.7,94.9z M54.7,100.5c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6C57.3,99.3,56.1,100.5,54.7,100.5z"></path>
        <path d="M63.6,94.9c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S65.3,94.9,63.6,94.9z M63.6,100.5c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6C66.3,99.3,65.1,100.5,63.6,100.5z"></path>
        <path d="M72.6,94.9c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S74.2,94.9,72.6,94.9z M72.6,100.5c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6C75.2,99.3,74,100.5,72.6,100.5z"></path>
        <path d="M81.6,94.9c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S83.2,94.9,81.6,94.9z M81.6,100.5c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6C84.2,99.3,83,100.5,81.6,100.5z"></path>
        <path d="M90.5,94.9c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S92.1,94.9,90.5,94.9z M90.5,100.5c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6C93.1,99.3,91.9,100.5,90.5,100.5z"></path>
        <path d="M99.5,94.9c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S101.1,94.9,99.5,94.9z M99.5,100.5c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6C102.1,99.3,100.9,100.5,99.5,100.5z"></path>
        <path d="M108.4,94.9c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S110.1,94.9,108.4,94.9z M108.4,100.5c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6C111,99.3,109.9,100.5,108.4,100.5z"></path>
        <path d="M117.4,94.9c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S119,94.9,117.4,94.9z M117.4,100.5c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6C120,99.3,118.8,100.5,117.4,100.5z"></path>
        <path d="M126.3,94.9c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S128,94.9,126.3,94.9z M126.3,100.5c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6C129,99.3,127.8,100.5,126.3,100.5z"></path>
        <path d="M135.3,94.9c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S137,94.9,135.3,94.9z M135.3,100.5c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6C137.9,99.3,136.8,100.5,135.3,100.5z"></path>
        <path d="M36.7,94.9c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S38.4,94.9,36.7,94.9z M36.7,100.5c-1.4,0-2.6-1.2-2.6-2.6 c0-1.4,1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6C39.4,99.3,38.2,100.5,36.7,100.5z"></path>
      </g>

      <g id="row_G_block_wells">
        <path d="M45.7,85.9c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S47.3,85.9,45.7,85.9z M45.7,91.5c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S47.2,91.5,45.7,91.5z"></path>
        <path d="M54.7,85.9c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S56.3,85.9,54.7,85.9z M54.7,91.5c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S56.1,91.5,54.7,91.5z"></path>
        <path d="M63.6,85.9c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S65.3,85.9,63.6,85.9z M63.6,91.5c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S65.1,91.5,63.6,91.5z"></path>
        <path d="M72.6,85.9c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S74.2,85.9,72.6,85.9z M72.6,91.5c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S74,91.5,72.6,91.5z"></path>
        <path d="M81.6,85.9c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S83.2,85.9,81.6,85.9z M81.6,91.5c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S83,91.5,81.6,91.5z"></path>
        <path d="M90.5,85.9c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S92.1,85.9,90.5,85.9z M90.5,91.5c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S91.9,91.5,90.5,91.5z"></path>
        <path d="M99.5,85.9c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S101.1,85.9,99.5,85.9z M99.5,91.5c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S100.9,91.5,99.5,91.5z"></path>
        <path d="M108.4,85.9c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S110.1,85.9,108.4,85.9z M108.4,91.5c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S109.9,91.5,108.4,91.5z"></path>
        <path d="M117.4,85.9c-1.6,0-3,1.3-3,3s1.3,3,3,3c1.6,0,3-1.3,3-3S119,85.9,117.4,85.9z M117.4,91.5c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S118.8,91.5,117.4,91.5z"></path>
        <path d="M126.3,85.9c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S128,85.9,126.3,85.9z M126.3,91.5c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S127.8,91.5,126.3,91.5z"></path>
        <path d="M135.3,85.9c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S137,85.9,135.3,85.9z M135.3,91.5c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S136.8,91.5,135.3,91.5z"></path>
        <path d="M36.7,85.9c-1.6,0-3,1.3-3,3s1.3,3,3,3s3-1.3,3-3S38.4,85.9,36.7,85.9z M36.7,91.5c-1.4,0-2.6-1.2-2.6-2.6 s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S38.2,91.5,36.7,91.5z"></path>
      </g>

      <g id="row_H_block_wells">
        <path d="M45.7,76.9c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3s3-1.3,3-3C48.7,78.2,47.3,76.9,45.7,76.9z M45.7,82.5 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S47.2,82.5,45.7,82.5z"></path>
        <path d="M54.7,76.9c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3c1.6,0,3-1.3,3-3C57.7,78.2,56.3,76.9,54.7,76.9z M54.7,82.5 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S56.1,82.5,54.7,82.5z"></path>
        <path d="M63.6,76.9c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3c1.6,0,3-1.3,3-3C66.6,78.2,65.3,76.9,63.6,76.9z M63.6,82.5 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S65.1,82.5,63.6,82.5z"></path>
        <path d="M72.6,76.9c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3c1.6,0,3-1.3,3-3C75.6,78.2,74.2,76.9,72.6,76.9z M72.6,82.5 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S74,82.5,72.6,82.5z"></path>
        <path d="M81.6,76.9c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3s3-1.3,3-3C84.5,78.2,83.2,76.9,81.6,76.9z M81.6,82.5 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S83,82.5,81.6,82.5z"></path>
        <path d="M90.5,76.9c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3s3-1.3,3-3C93.5,78.2,92.1,76.9,90.5,76.9z M90.5,82.5 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S91.9,82.5,90.5,82.5z"></path>
        <path d="M99.5,76.9c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3s3-1.3,3-3C102.4,78.2,101.1,76.9,99.5,76.9z M99.5,82.5 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S100.9,82.5,99.5,82.5z"></path>
        <path d="M108.4,76.9c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3s3-1.3,3-3C111.4,78.2,110.1,76.9,108.4,76.9z M108.4,82.5 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S109.9,82.5,108.4,82.5z"></path>
        <path d="M117.4,76.9c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3c1.6,0,3-1.3,3-3C120.3,78.2,119,76.9,117.4,76.9z M117.4,82.5 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S118.8,82.5,117.4,82.5z"></path>
        <path d="M126.3,76.9c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3s3-1.3,3-3C129.3,78.2,128,76.9,126.3,76.9z M126.3,82.5 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S127.8,82.5,126.3,82.5z"></path>
        <path d="M135.3,76.9c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3s3-1.3,3-3C138.3,78.2,137,76.9,135.3,76.9z M135.3,82.5 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6c1.4,0,2.6,1.2,2.6,2.6S136.8,82.5,135.3,82.5z"></path>
        <path d="M36.7,76.9c-1.6,0-3,1.3-3,3c0,1.6,1.3,3,3,3s3-1.3,3-3C39.7,78.2,38.4,76.9,36.7,76.9z M36.7,82.5 c-1.4,0-2.6-1.2-2.6-2.6s1.2-2.6,2.6-2.6s2.6,1.2,2.6,2.6S38.2,82.5,36.7,82.5z"></path>
      </g>
      <g id="row8">
        <path d="M171.6,208.8H0.4c-0.1,0-0.2,0.1-0.2,0.2c0,0.1,0.1,0.2,0.2,0.2h171.3c0.1,0,0.2-0.1,0.2-0.2 C171.8,208.9,171.7,208.8,171.6,208.8z"></path>
        <path d="M171.6,212.8H0.4c-0.1,0-0.2,0.1-0.2,0.2c0,0.1,0.1,0.2,0.2,0.2h171.3c0.1,0,0.2-0.1,0.2-0.2 C171.8,212.9,171.7,212.8,171.6,212.8z"></path>
        <path d="M10.9,202.8h-0.4c-0.1,0-0.2,0.1-0.2,0.2s0.1,0.2,0.2,0.2h0.4c0.3,0,0.6,0.1,0.8,0.4l3.3,4.5c0.4,0.5,0.8,0.9,1.3,1.1 c0.1,0,0.2,0,0.2-0.1c0-0.1,0-0.2-0.1-0.2c-0.5-0.2-0.9-0.6-1.2-1l-3.3-4.5C11.7,203,11.3,202.8,10.9,202.8z"></path>
        <path d="M161.6,202.8h-0.4c-0.4,0-0.8,0.2-1.1,0.5l-3.3,4.5c-0.3,0.4-0.7,0.8-1.2,1c-0.1,0-0.1,0.1-0.1,0.2 c0,0.1,0.1,0.1,0.2,0.1c0.5-0.2,1-0.6,1.3-1.1l3.3-4.5c0.2-0.3,0.5-0.4,0.8-0.4h0.4c0.1,0,0.2-0.1,0.2-0.2 C161.8,202.9,161.7,202.8,161.6,202.8z"></path>
        <path d="M161.6,202.7H10.5c-0.2,0-0.4,0.2-0.4,0.4s0.2,0.4,0.4,0.4h151.2c0.2,0,0.4-0.2,0.4-0.4S161.8,202.7,161.6,202.7z"></path>
        <path d="M0.4,103.9H0.3c-0.1,0-0.1,0.1-0.1,0.1c0,0.1,0.1,0.1,0.1,0.1h0.1c0.1,0,0.1-0.1,0.1-0.1C0.5,104,0.5,103.9,0.4,103.9z"></path>
        <path d="M167.6,38.9c-0.1,0-0.2,0.1-0.2,0.2v203.4H4.6V39.1c0-0.1-0.1-0.2-0.2-0.2c-0.1,0-0.2,0.1-0.2,0.2v203.6 c0,0.1,0.1,0.2,0.2,0.2h163.2c0.1,0,0.2-0.1,0.2-0.2V39.1C167.8,39,167.7,38.9,167.6,38.9z"></path>
        <path d="M150.5,63.9h-129c-1.4,0-2.6,1.2-2.6,2.6v4.4c0,0.2,0.2,0.4,0.4,0.4h5.9c0.4,0,0.6,0.3,0.6,0.6v2.5 c0,0.4-0.3,0.6-0.6,0.6h-5.9c-0.2,0-0.4,0.2-0.4,0.4v65.3c0,0.2,0.2,0.4,0.4,0.4h5.9c0.4,0,0.6,0.3,0.6,0.6v2.5 c0,0.4-0.3,0.6-0.6,0.6h-5.9c-0.2,0-0.4,0.2-0.4,0.4V156c0,1.4,1.2,2.6,2.6,2.6h129c1.4,0,2.6-1.2,2.6-2.6v-10.9 c0-0.2-0.2-0.4-0.4-0.4h-5.9c-0.4,0-0.6-0.3-0.6-0.6v-2.5c0-0.4,0.3-0.6,0.6-0.6h5.9c0.2,0,0.4-0.2,0.4-0.4V75.4 c0-0.2-0.2-0.4-0.4-0.4h-5.9c-0.4,0-0.6-0.3-0.6-0.6v-2.5c0-0.4,0.3-0.6,0.6-0.6h5.9c0.2,0,0.4-0.2,0.4-0.4v-4.4 C153.1,65.1,151.9,63.9,150.5,63.9z M19.7,70.5v-4c0-1,0.8-1.9,1.9-1.9h129c1,0,1.9,0.8,1.9,1.9v4h-5.5c-0.7,0-1.3,0.6-1.3,1.3 v2.5c0,0.7,0.6,1.3,1.3,1.3h5.5v64.6h-5.5c-0.7,0-1.3,0.6-1.3,1.3v2.5c0,0.7,0.6,1.3,1.3,1.3h5.5V156c0,1-0.8,1.9-1.9,1.9h-129 c-1,0-1.9-0.8-1.9-1.9v-10.5h5.5c0.7,0,1.3-0.6,1.3-1.3v-2.5c0-0.7-0.6-1.3-1.3-1.3h-5.5V75.7h5.5c0.7,0,1.3-0.6,1.3-1.3v-2.5 c0-0.7-0.6-1.3-1.3-1.3L19.7,70.5L19.7,70.5z"></path>
        <path d="M47.7,52.7c-3.2,0-5.2,3.4-6.3,5.5c-0.1,0.1-0.1,0.2-0.2,0.3c-0.4,0.8-0.9,1.7-1.8,1.7H21c-3.5,0-6.3,2.8-6.3,6.3v95.9 c0,3.5,2.8,6.3,6.3,6.3h130c3.5,0,6.3-2.8,6.3-6.3V66.6c0-3.5-2.8-6.3-6.3-6.3h-18.4c-0.9,0-1.3-0.9-1.8-1.7 c-0.1-0.1-0.1-0.2-0.2-0.3c-1-1.9-2.2-3.9-4.2-5c-0.8-0.4-1.7-0.6-2.5-0.5H48.1C47.9,52.7,47.8,52.7,47.7,52.7z M21,168.1 c-3.1,0-5.6-2.5-5.6-5.6V66.6c0-3.1,2.5-5.6,5.6-5.6h18.4c1.3,0,1.9-1.2,2.4-2.1c0.1-0.1,0.1-0.2,0.2-0.3 c1.5-2.7,3.2-5.4,6.1-5.1h76c0,0,0,0,0,0c0.7-0.1,1.5,0.1,2.2,0.4c1.8,0.9,2.9,2.9,3.9,4.7c0.1,0.1,0.1,0.2,0.2,0.3 c0.5,0.9,1.1,2.1,2.4,2.1H151c3.1,0,5.6,2.5,5.6,5.6v95.9c0,3.1-2.5,5.6-5.6,5.6H21z"></path>
        <path d="M141,191.4H31.1c-1.3,0-2.3,1.1-2.3,2.3v9.2c0,0.2,0.2,0.4,0.4,0.4c0.2,0,0.4-0.2,0.4-0.4v-9.2c0-0.9,0.7-1.6,1.6-1.6H141 c0.9,0,1.6,0.7,1.6,1.6v9.2c0,0.2,0.2,0.4,0.4,0.4s0.4-0.2,0.4-0.4v-9.2C143.3,192.5,142.3,191.4,141,191.4z"></path>
        <path d="M139,195.1H33.1c-0.4,0-0.7,0.3-0.7,0.7v7.5c0,0.1,0.1,0.2,0.2,0.2c0.1,0,0.2-0.1,0.2-0.2v-7.5c0-0.2,0.1-0.3,0.3-0.3H139 c0.2,0,0.3,0.1,0.3,0.3v7.4c0,0.1,0.1,0.2,0.2,0.2c0.1,0,0.2-0.1,0.2-0.2v-7.4C139.7,195.4,139.4,195.1,139,195.1z"></path>
      </g>
    </g>
  )
}
