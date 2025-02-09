import styled from 'styled-components'
import {
  SPACING,
  COLORS,
  TYPOGRAPHY,
  ALIGN_CENTER,
  RESPONSIVENESS,
} from '@opentrons/components'
import type { StyleProps } from '@opentrons/components'

interface ButtonProps extends StyleProps {
  /** optional isAlert boolean to turn the background red, only seen in ODD */
  isAlert?: boolean
}
export const MenuItem = styled.button<ButtonProps>`
  text-align: ${TYPOGRAPHY.textAlignLeft};
  font-size: ${TYPOGRAPHY.fontSizeP};
  background-color: ${COLORS.transparent};
  color: ${COLORS.black90};
  padding: ${SPACING.spacing8} ${SPACING.spacing12} ${SPACING.spacing8}
    ${SPACING.spacing12};

  &:hover {
    background-color: ${COLORS.blue10};
  }

  &:disabled {
    background-color: ${COLORS.transparent};
    color: ${COLORS.grey40};
  }

  @media ${RESPONSIVENESS.touchscreenMediaQuerySpecs} {
    align-items: ${ALIGN_CENTER};
    text-align: ${TYPOGRAPHY.textAlignCenter};
    font-size: ${TYPOGRAPHY.fontSize28};
    background-color: ${({ isAlert }) =>
      isAlert != null ? COLORS.red50 : COLORS.transparent};
    color: ${({ isAlert }) =>
      isAlert != null ? COLORS.white : COLORS.black90};
    padding: ${SPACING.spacing24};
    height: 5.5rem;
    line-height: ${TYPOGRAPHY.lineHeight36};
    &:hover,
    &:active {
      background-color: ${({ isAlert }) =>
        isAlert != null ? COLORS.red50 : COLORS.grey35};
    }

    &:disabled {
      background-color: ${({ isAlert }) =>
        isAlert != null ? COLORS.red50 : COLORS.transparent};
      color: ${({ isAlert }) =>
        isAlert != null ? COLORS.white : COLORS.grey50};
    }
  }
`
