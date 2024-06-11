import * as React from 'react'
import { useTranslation } from 'react-i18next'
import without from 'lodash/without'
import { Flex, POSITION_FIXED, SPACING } from '@opentrons/components'

import { ChildNavigation } from '../ChildNavigation'
import { WellSelection } from '../../organisms/WellSelection'

import type { SmallButton } from '../../atoms/buttons'
import type {
  QuickTransferWizardState,
  QuickTransferWizardAction,
} from './types'

interface SelectDestWellsProps {
  onNext: () => void
  onBack: () => void
  state: QuickTransferWizardState
  dispatch: React.Dispatch<QuickTransferWizardAction>
}

export function SelectDestWells(props: SelectDestWellsProps): JSX.Element {
  const { onNext, onBack, state, dispatch } = props
  const { i18n, t } = useTranslation(['quick_transfer', 'shared'])

  const destinationWells = state.destinationWells ?? []
  const destinationWellGroup = destinationWells.reduce((acc, well) => {
    return { ...acc, [well]: null }
  }, {})

  const [selectedWells, setSelectedWells] = React.useState(destinationWellGroup)

  const handleClickNext = (): void => {
    dispatch({
      type: 'SET_DEST_WELLS',
      wells: Object.keys(selectedWells),
    })
    onNext()
  }

  const resetButtonProps: React.ComponentProps<typeof SmallButton> = {
    buttonType: 'tertiaryLowLight',
    buttonText: t('shared:reset'),
    onClick: () => {
      setSelectedWells({})
    },
  }

  return (
    <>
      <ChildNavigation
        header={t('select_dest_wells')}
        onClickBack={onBack}
        buttonText={i18n.format(t('shared:continue'), 'capitalize')}
        onClickButton={handleClickNext}
        buttonIsDisabled={false}
        secondaryButtonProps={resetButtonProps}
        top={SPACING.spacing8}
      />
      <Flex
        marginTop={SPACING.spacing120}
        padding={`${SPACING.spacing16} ${SPACING.spacing60} ${SPACING.spacing40} ${SPACING.spacing60}`}
        position={POSITION_FIXED}
        top="0"
        left="0"
        width="100%"
      >
        {state.destination != null && state.source != null ? (
          <WellSelection
            definition={
              state.destination === 'source' ? state.source : state.destination
            }
            deselectWells={(wells: string[]) => {
              setSelectedWells(prevWells =>
                without(Object.keys(prevWells), ...wells).reduce(
                  (acc, well) => {
                    return { ...acc, [well]: null }
                  },
                  {}
                )
              )
            }}
            selectedPrimaryWells={selectedWells}
            selectWells={wellGroup => {
              setSelectedWells(prevWells => ({ ...prevWells, ...wellGroup }))
            }}
            channels={state.pipette?.channels ?? 1}
          />
        ) : null}
      </Flex>
    </>
  )
}
