import takeWhile from 'lodash/takeWhile'
import * as StepGeneration from '@opentrons/step-generation'
import { commandCreatorFromStepArgs } from '../file-data/selectors/commands'
import type { StepArgsAndErrorsById } from '../steplist/types'
export interface GenerateRobotStateTimelineArgs {
  allStepArgsAndErrors: StepArgsAndErrorsById
  orderedStepIds: string[]
  initialRobotState: StepGeneration.RobotState
  invariantContext: StepGeneration.InvariantContext
}
export const generateRobotStateTimeline = (
  args: GenerateRobotStateTimelineArgs
): StepGeneration.Timeline => {
  const {
    allStepArgsAndErrors,
    orderedStepIds,
    initialRobotState,
    invariantContext,
  } = args
  const allStepArgs: Array<StepGeneration.CommandCreatorArgs | null> = orderedStepIds.map(
    stepId => {
      return (
        (allStepArgsAndErrors[stepId] &&
          allStepArgsAndErrors[stepId].stepArgs) ||
        null
      )
    }
  )
  // @ts-expect-error(sa, 2021-7-6): stepArgs might be null (see code above). this was incorrectly typed from before the TS migration and requires source code changes
  const continuousStepArgs: StepGeneration.CommandCreatorArgs[] = takeWhile(
    allStepArgs,
    stepArgs => stepArgs
  )
  const curriedCommandCreators = continuousStepArgs.reduce(
    (
      acc: StepGeneration.CurriedCommandCreator[],
      args: StepGeneration.CommandCreatorArgs,
      stepIndex
    ): StepGeneration.CurriedCommandCreator[] => {
      const curriedCommandCreator = commandCreatorFromStepArgs(args)

      if (curriedCommandCreator === null) {
        // unsupported command creator in args.commandCreatorFnName
        return acc
      }

      // Drop tips eagerly, per pipette
      //
      // - If we don't have a 'changeTip: never' step for this pipette in the future,
      // we know the current tip(s) aren't going to be reused, so we can drop them
      // immediately after the current step is done.
      const pipetteId = StepGeneration.getPipetteIdFromCCArgs(args)
      const dropTipLocation =
        'dropTipLocation' in args ? args.dropTipLocation : null

      //  assume that whenever we have a pipetteId we also have a dropTipLocation
      if (pipetteId != null && dropTipLocation != null) {
        const nextStepArgsForPipette = continuousStepArgs
          .slice(stepIndex + 1)
          .find(stepArgs => 'pipette' in stepArgs && stepArgs.pipette === pipetteId)
        const willReuseTip =
          nextStepArgsForPipette != null &&
          'changeTip' in nextStepArgsForPipette &&
          nextStepArgsForPipette.changeTip === 'never'

        if (!willReuseTip) {
          return [
            ...acc,
            (_invariantContext, _prevRobotState) =>
              StepGeneration.reduceCommandCreators(
                [
                  curriedCommandCreator,
                  StepGeneration.curryCommandCreator(StepGeneration.dropTip, {
                    pipette: pipetteId,
                    dropTipLocation,
                  }),
                ],
                _invariantContext,
                _prevRobotState
              ),
          ]
        }
      }

      return [...acc, curriedCommandCreator]
    },
    []
  )
  const timeline = StepGeneration.commandCreatorsTimeline(
    curriedCommandCreators,
    invariantContext,
    initialRobotState
  )
  return timeline
}
