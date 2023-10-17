import { useSelector } from 'react-redux'
import * as React from 'react'
import { useTranslation } from 'react-i18next'
import {
  COLORS,
  PrimaryButton,
  TEXT_TRANSFORM_CAPITALIZE,
  JUSTIFY_FLEX_END,
  Flex,
} from '@opentrons/components'
import { getIsOnDevice } from '../../redux/config'
import { SimpleWizardBody } from '../../molecules/SimpleWizardBody'
import { InProgressModal } from '../../molecules/InProgressModal/InProgressModal'
import { SmallButton } from '../../atoms/buttons'
import { BLOWOUT_SUCCESS } from './constants'

interface SuccessProps {
  message: string
  proceedText: string
  handleProceed: () => void
  isRobotMoving: boolean
  isExiting: boolean
  currentStep: string
}
export const Success = (props: SuccessProps): JSX.Element => {
  const {
    message,
    proceedText,
    handleProceed,
    isRobotMoving,
    isExiting,
    currentStep,
  } = props
  const isOnDevice = useSelector(getIsOnDevice)

  const { i18n, t } = useTranslation(['drop_tip_wizard', 'shared'])

  return isRobotMoving && !isExiting ? (
    <InProgressModal
      alternativeSpinner={null}
      description={t('stand_back_exiting')}
    />
  ) : (
    <SimpleWizardBody
      iconColor={COLORS.successEnabled}
      header={i18n.format(message, 'capitalize')}
      isSuccess
    >
      {isOnDevice ? (
        <Flex justifyContent={JUSTIFY_FLEX_END} width="100%">
          <SmallButton
            textTransform={TEXT_TRANSFORM_CAPITALIZE}
            buttonText={proceedText}
            onClick={handleProceed}
          />
        </Flex>
      ) : (
        <PrimaryButton onClick={handleProceed}>{proceedText}</PrimaryButton>
      )}
    </SimpleWizardBody>
  )
}
