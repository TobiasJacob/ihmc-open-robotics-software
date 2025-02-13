package us.ihmc.commonWalkingControlModules.controlModules;

import static us.ihmc.communication.packets.Packet.INVALID_MESSAGE_ID;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.lists.RecyclingArrayDeque;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector2D;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.listener.YoParameterChangedListener;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.YoParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class PelvisICPBasedTranslationManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoDouble supportPolygonSafeMargin = new YoDouble("supportPolygonSafeMargin", registry);
   private final DoubleProvider frozenOffsetDecayBreakFrequency = new DoubleParameter("frozenOffsetDecayBreakFrequency", registry, 0.0531);
   private final double dt;

   private final YoFramePoint2D desiredPelvisPosition = new YoFramePoint2D("desiredPelvis", worldFrame, registry);
   private final YoFramePoint2D currentPelvisPosition = new YoFramePoint2D("currentPelvis", worldFrame, registry);

   private final YoDouble initialPelvisPositionTime = new YoDouble("initialPelvisPositionTime", registry);

   private final MultipleWaypointsPositionTrajectoryGenerator positionTrajectoryGenerator;

   private final YoFrameVector2D pelvisPositionError = new YoFrameVector2D("pelvisPositionError", worldFrame, registry);
   private final YoDouble proportionalGain = new YoDouble("pelvisPositionProportionalGain", registry);
   private final YoFrameVector2D proportionalTerm = new YoFrameVector2D("pelvisPositionProportionalTerm", worldFrame, registry);

   private final YoFrameVector2D pelvisPositionCumulatedError = new YoFrameVector2D("pelvisPositionCumulatedError", worldFrame, registry);
   private final YoDouble integralGain = new YoDouble("pelvisPositionIntegralGain", registry);
   private final YoFrameVector2D integralTerm = new YoFrameVector2D("pelvisPositionIntegralTerm", worldFrame, registry);
   private final YoDouble maximumIntegralError = new YoDouble("maximumPelvisPositionIntegralError", registry);

   private final YoFrameVector2D desiredICPOffset = new YoFrameVector2D("desiredICPOffset", worldFrame, registry);
   private final Vector2DReadOnly userOffset = new ParameterVector2D("userDesiredICPOffset", new Vector2D(), registry);

   private final YoBoolean isEnabled = new YoBoolean("isPelvisTranslationManagerEnabled", registry);
   private final YoBoolean isRunning = new YoBoolean("isPelvisTranslationManagerRunning", registry);
   private final YoBoolean isFrozen = new YoBoolean("isPelvisTranslationManagerFrozen", registry);

   private final BooleanParameter manualMode = new BooleanParameter("manualModeICPOffset", registry, false);

   private final YoDouble yoTime;
   private final double controlDT;

   private final YoBoolean isTrajectoryStopped = new YoBoolean("isPelvisTranslationalTrajectoryStopped", registry);

   private ReferenceFrame supportFrame;
   private final ReferenceFrame pelvisZUpFrame;
   private final ReferenceFrame midFeetZUpFrame;
   private final SideDependentList<MovingReferenceFrame> soleZUpFrames;

   private final BipedSupportPolygons bipedSupportPolygons;
   private FrameConvexPolygon2DReadOnly supportPolygon;

   private final FramePoint3D tempPosition = new FramePoint3D();
   private final FrameVector3D tempVelocity = new FrameVector3D();
   private final FramePoint2D tempPosition2d = new FramePoint2D();
   private final FrameVector2D tempError2d = new FrameVector2D();
   private final FrameVector2D tempICPOffset = new FrameVector2D();
   private final FrameVector2D icpOffsetForFreezing = new FrameVector2D();

   private final YoLong lastCommandId;

   private final YoBoolean isReadyToHandleQueuedCommands;
   private final YoLong numberOfQueuedCommands;
   private final PelvisTrajectoryCommand commandBeingProcessed = new PelvisTrajectoryCommand();
   private final RecyclingArrayDeque<PelvisTrajectoryCommand> commandQueue = new RecyclingArrayDeque<>(PelvisTrajectoryCommand.class,
                                                                                                       PelvisTrajectoryCommand::set);
   /**
    * Used when streaming to account for time variations occurring during the transport of the message
    * over the network.
    */
   private final YoDouble streamTimestampOffset = new YoDouble("pelvisTranslationStreamTimestampOffset", registry);
   private final YoDouble streamTimestampSource = new YoDouble("pelvisTranslationStreamTimestampSource", registry);
   private final TaskspaceTrajectoryStatusMessageHelper statusHelper = new TaskspaceTrajectoryStatusMessageHelper("pelvisXY");

   public PelvisICPBasedTranslationManager(HighLevelHumanoidControllerToolbox controllerToolbox,
                                           double pelvisTranslationICPSupportPolygonSafeMargin,
                                           BipedSupportPolygons bipedSupportPolygons,
                                           YoRegistry parentRegistry)
   {
      dt = controllerToolbox.getControlDT();

      supportPolygonSafeMargin.set(pelvisTranslationICPSupportPolygonSafeMargin);

      yoTime = controllerToolbox.getYoTime();
      controlDT = controllerToolbox.getControlDT();
      pelvisZUpFrame = controllerToolbox.getPelvisZUpFrame();
      midFeetZUpFrame = controllerToolbox.getReferenceFrames().getMidFeetZUpFrame();
      soleZUpFrames = controllerToolbox.getReferenceFrames().getSoleZUpFrames();

      this.bipedSupportPolygons = bipedSupportPolygons;

      positionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator("pelvisOffset",
                                                                                     RigidBodyTaskspaceControlState.maxPointsInGenerator,
                                                                                     worldFrame,
                                                                                     registry);

      proportionalGain.set(0.5);
      integralGain.set(1.5);
      maximumIntegralError.set(0.08);

      manualMode.addListener(new YoParameterChangedListener()
      {
         @Override
         public void changed(YoParameter v)
         {
            initialize();
         }
      });

      String namePrefix = "PelvisXYTranslation";
      lastCommandId = new YoLong(namePrefix + "LastCommandId", registry);
      lastCommandId.set(Packet.INVALID_MESSAGE_ID);

      isReadyToHandleQueuedCommands = new YoBoolean(namePrefix + "IsReadyToHandleQueuedPelvisTrajectoryCommands", registry);
      numberOfQueuedCommands = new YoLong(namePrefix + "NumberOfQueuedCommands", registry);

      streamTimestampOffset.setToNaN();
      streamTimestampSource.setToNaN();

      parentRegistry.addChild(registry);
   }

   public void compute(RobotSide supportLeg)
   {
      tempPosition2d.setToZero(pelvisZUpFrame);
      tempPosition2d.changeFrame(worldFrame);
      currentPelvisPosition.set(tempPosition2d);

      if (isFrozen.getBooleanValue())
      {
         double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(frozenOffsetDecayBreakFrequency.getValue(), dt);
         icpOffsetForFreezing.scale(alpha);
         return;
      }

      if (supportLeg == null)
      {
         supportFrame = midFeetZUpFrame;
         supportPolygon = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
      }
      else
      {
         supportFrame = soleZUpFrames.get(supportLeg);
         supportPolygon = bipedSupportPolygons.getFootPolygonInSoleZUpFrame(supportLeg);
      }

      if (!isEnabled.getBooleanValue())
      {
         desiredICPOffset.setToZero();
         return;
      }

      if (manualMode.getValue())
         return;

      if (isRunning.getBooleanValue())
      {
         if (!isTrajectoryStopped.getBooleanValue())
         {
            double deltaTime = yoTime.getDoubleValue() - initialPelvisPositionTime.getDoubleValue();
            statusHelper.updateWithTimeInTrajectory(deltaTime);
            positionTrajectoryGenerator.compute(deltaTime);

            if (positionTrajectoryGenerator.isDone())
            {
               if (!commandQueue.isEmpty())
               {
                  double firstTrajectoryPointTime = positionTrajectoryGenerator.getLastWaypointTime();
                  commandBeingProcessed.set(commandQueue.poll());
                  numberOfQueuedCommands.decrement();
                  initializeTrajectoryGenerator(commandBeingProcessed, firstTrajectoryPointTime);
                  positionTrajectoryGenerator.compute(deltaTime);

                  if (positionTrajectoryGenerator.isDone())
                  {
                     streamTimestampOffset.setToNaN();
                     streamTimestampSource.setToNaN();
                  }
               }
               else
               {
                  streamTimestampOffset.setToNaN();
                  streamTimestampSource.setToNaN();
               }
            }
         }
         tempPosition.setIncludingFrame(positionTrajectoryGenerator.getPosition());
         tempPosition.changeFrame(desiredPelvisPosition.getReferenceFrame());
         desiredPelvisPosition.set(tempPosition);
      }

      if (!isRunning.getBooleanValue())
      {
         desiredICPOffset.setToZero();
         return;
      }

      computeDesiredICPOffset();
   }

   public boolean isEnabled()
   {
      return isEnabled.getBooleanValue();
   }

   public void goToHome()
   {
      freeze();
   }

   public void holdCurrentPosition()
   {
      initialPelvisPositionTime.set(yoTime.getDoubleValue());

      tempPosition.setToZero(pelvisZUpFrame);
      tempPosition.changeFrame(worldFrame);
      tempVelocity.setToZero(worldFrame);

      positionTrajectoryGenerator.clear();
      positionTrajectoryGenerator.changeFrame(worldFrame);
      positionTrajectoryGenerator.appendWaypoint(0.0, tempPosition, tempVelocity);
      positionTrajectoryGenerator.initialize();
      isTrajectoryStopped.set(false);
      streamTimestampOffset.setToNaN();
      streamTimestampSource.setToNaN();
      isRunning.set(true);
   }

   private final FrameSE3TrajectoryPoint trajectoryPointLocal = new FrameSE3TrajectoryPoint();

   public void handlePelvisTrajectoryCommand(PelvisTrajectoryCommand command)
   {
      SE3TrajectoryControllerCommand se3Trajectory = command.getSE3Trajectory();
      SelectionMatrix3D linearSelectionMatrix = se3Trajectory.getSelectionMatrix().getLinearPart();

      if (!linearSelectionMatrix.isXSelected() && !linearSelectionMatrix.isYSelected())
         return; // The user does not want to control the x and y of the pelvis, do nothing.

      se3Trajectory.setSequenceId(command.getSequenceId());

      if (se3Trajectory.getExecutionMode() == ExecutionMode.OVERRIDE)
      {
         isReadyToHandleQueuedCommands.set(true);
         clearCommandQueue(se3Trajectory.getCommandId());
         initialPelvisPositionTime.set(yoTime.getDoubleValue());
         initializeTrajectoryGenerator(command, 0.0);
         streamTimestampOffset.setToNaN();
         streamTimestampSource.setToNaN();
         statusHelper.registerNewTrajectory(se3Trajectory);
         return;
      }
      else if (se3Trajectory.getExecutionMode() == ExecutionMode.QUEUE)
      {
         boolean success = queuePelvisTrajectoryCommand(command);
         if (!success)
         {
            isReadyToHandleQueuedCommands.set(false);
            clearCommandQueue(INVALID_MESSAGE_ID);
            holdCurrentPosition();
         }
         else
         {
            statusHelper.registerNewTrajectory(se3Trajectory);
         }

         streamTimestampOffset.setToNaN();
         streamTimestampSource.setToNaN();

         return;
      }
      else if (se3Trajectory.getExecutionMode() == ExecutionMode.STREAM)
      {
         double timeOffset = 0.0;

         // Need to do time checks before moving on.
         if (se3Trajectory.getTimestamp() <= 0)
         {
            streamTimestampOffset.setToNaN();
            streamTimestampSource.setToNaN();
         }
         else
         {
            double senderTime = Conversions.nanosecondsToSeconds(se3Trajectory.getTimestamp());

            if (!streamTimestampSource.isNaN() && senderTime < streamTimestampSource.getValue())
            {
               // Messages are out of order which is fine, we just don't want to handle the new message.
               return;
            }

            streamTimestampSource.set(senderTime);

            timeOffset = yoTime.getValue() - senderTime;

            if (Double.isNaN(streamTimestampOffset.getValue()))
            {
               streamTimestampOffset.set(timeOffset);
            }
            else
            {
               /*
                * Update to the smallest time offset, which is closer to the true offset between the sender CPU and
                * control CPU. If the change in offset is too large though, we always set the streamTimestampOffset
                * for safety.
                */
               if (Math.abs(timeOffset - streamTimestampOffset.getValue()) > 0.5)
                  streamTimestampOffset.set(timeOffset);
               else
                  streamTimestampOffset.set(Math.min(timeOffset, streamTimestampOffset.getValue()));
            }
         }

         isReadyToHandleQueuedCommands.set(true);
         clearCommandQueue(se3Trajectory.getCommandId());
         initialPelvisPositionTime.set(yoTime.getDoubleValue());

         if (se3Trajectory.getNumberOfTrajectoryPoints() != 1)
         {
            LogTools.warn("When streaming, trajectories should contain only 1 trajectory point, was: " + se3Trajectory.getNumberOfTrajectoryPoints());
            holdCurrentPosition();
            return;
         }

         FrameSE3TrajectoryPoint trajectoryPoint = se3Trajectory.getTrajectoryPoint(0);

         if (trajectoryPoint.getTime() != 0.0)
         {
            LogTools.warn("When streaming, the trajectory point should have a time of zero, was: " + trajectoryPoint.getTime());
            holdCurrentPosition();
            return;
         }

         // Applying the time offset to the trajectory point:
         trajectoryPointLocal.setIncludingFrame(trajectoryPoint);
         // When a message is delayed during shipping over network, timeOffset > streamTimestampOffset.getValue().
         // This new time will put the trajectory point in the past, closer to when it should have been received.
         if (!streamTimestampOffset.isNaN())
            trajectoryPointLocal.setTime(streamTimestampOffset.getValue() - timeOffset);

         positionTrajectoryGenerator.clear();
         positionTrajectoryGenerator.changeFrame(worldFrame);
         positionTrajectoryGenerator.appendWaypoint(trajectoryPointLocal);
         tempPosition.setIncludingFrame(trajectoryPointLocal.getLinearVelocity());
         tempPosition.scaleAdd(se3Trajectory.getStreamIntegrationDuration(), trajectoryPoint.getPosition());
         positionTrajectoryGenerator.appendWaypoint(se3Trajectory.getStreamIntegrationDuration() + trajectoryPointLocal.getTime(),
                                                    tempPosition,
                                                    trajectoryPoint.getLinearVelocity());

         if (supportFrame != null)
            positionTrajectoryGenerator.changeFrame(supportFrame);
         else
            positionTrajectoryGenerator.changeFrame(worldFrame);

         positionTrajectoryGenerator.initialize();
         isTrajectoryStopped.set(false);
         isRunning.set(true);
         if (se3Trajectory.getExecutionMode() != ExecutionMode.STREAM)
            statusHelper.registerNewTrajectory(se3Trajectory);
         return;
      }
      else
      {
         LogTools.warn("Unknown " + ExecutionMode.class.getSimpleName() + " value: " + se3Trajectory.getExecutionMode() + ". Command ignored.");
      }
   }

   private boolean queuePelvisTrajectoryCommand(PelvisTrajectoryCommand command)
   {
      if (!isReadyToHandleQueuedCommands.getBooleanValue())
      {
         LogTools.warn("The very first " + command.getClass().getSimpleName() + " of a series must be " + ExecutionMode.OVERRIDE + ". Aborting motion.");
         return false;
      }

      SE3TrajectoryControllerCommand se3Trajectory = command.getSE3Trajectory();
      long previousCommandId = se3Trajectory.getPreviousCommandId();

      if (previousCommandId != INVALID_MESSAGE_ID && lastCommandId.getLongValue() != INVALID_MESSAGE_ID && lastCommandId.getLongValue() != previousCommandId)
      {
         LogTools.warn("Previous command ID mismatch: previous ID from command = " + previousCommandId + ", last message ID received by the controller = "
               + lastCommandId.getLongValue() + ". Aborting motion.");
         return false;
      }

      if (se3Trajectory.getTrajectoryPoint(0).getTime() < 1.0e-5)
      {
         LogTools.warn("Time of the first trajectory point of a queued command must be greater than zero. Aborting motion.");
         return false;
      }

      commandQueue.add(command);
      numberOfQueuedCommands.increment();
      lastCommandId.set(se3Trajectory.getCommandId());

      return true;
   }

   private void initializeTrajectoryGenerator(PelvisTrajectoryCommand command, double firstTrajectoryPointTime)
   {
      SE3TrajectoryControllerCommand se3Trajectory = command.getSE3Trajectory();
      se3Trajectory.addTimeOffset(firstTrajectoryPointTime);

      if (se3Trajectory.getTrajectoryPoint(0).getTime() > 1.0e-5)
      {
         if (isRunning.getBooleanValue())
            tempPosition.setIncludingFrame(positionTrajectoryGenerator.getPosition());
         else
            tempPosition.setToZero(pelvisZUpFrame);
         tempPosition.changeFrame(worldFrame);
         tempVelocity.setToZero(worldFrame);

         positionTrajectoryGenerator.clear();
         positionTrajectoryGenerator.changeFrame(worldFrame);
         positionTrajectoryGenerator.appendWaypoint(0.0, tempPosition, tempVelocity);
      }
      else
      {
         positionTrajectoryGenerator.clear();
         positionTrajectoryGenerator.changeFrame(worldFrame);
      }

      int numberOfTrajectoryPoints = queueExceedingTrajectoryPointsIfNeeded(command);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         positionTrajectoryGenerator.appendWaypoint(se3Trajectory.getTrajectoryPoint(trajectoryPointIndex));
      }

      if (supportFrame != null)
         positionTrajectoryGenerator.changeFrame(supportFrame);
      else
         positionTrajectoryGenerator.changeFrame(worldFrame);

      positionTrajectoryGenerator.initialize();
      isTrajectoryStopped.set(false);
      isRunning.set(true);
   }

   private int queueExceedingTrajectoryPointsIfNeeded(PelvisTrajectoryCommand command)
   {
      int numberOfTrajectoryPoints = command.getSE3Trajectory().getNumberOfTrajectoryPoints();

      int maximumNumberOfWaypoints = positionTrajectoryGenerator.getMaximumNumberOfWaypoints() - positionTrajectoryGenerator.getCurrentNumberOfWaypoints();

      if (numberOfTrajectoryPoints <= maximumNumberOfWaypoints)
         return numberOfTrajectoryPoints;

      PelvisTrajectoryCommand commandForExcedent = commandQueue.addFirst();
      numberOfQueuedCommands.increment();
      commandForExcedent.clear();
      commandForExcedent.getSE3Trajectory().setPropertiesOnly(command.getSE3Trajectory());

      for (int trajectoryPointIndex = maximumNumberOfWaypoints; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         commandForExcedent.getSE3Trajectory().addTrajectoryPoint(command.getSE3Trajectory().getTrajectoryPoint(trajectoryPointIndex));
      }

      double timeOffsetToSubtract = command.getSE3Trajectory().getTrajectoryPoint(maximumNumberOfWaypoints - 1).getTime();
      commandForExcedent.getSE3Trajectory().subtractTimeOffset(timeOffsetToSubtract);

      return maximumNumberOfWaypoints;
   }

   private void clearCommandQueue(long lastCommandId)
   {
      commandQueue.clear();
      numberOfQueuedCommands.set(0);
      this.lastCommandId.set(lastCommandId);
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      isTrajectoryStopped.set(command.isStopAllTrajectory());
   }

   private void computeDesiredICPOffset()
   {
      pelvisPositionError.set(desiredPelvisPosition);
      tempPosition2d.setToZero(pelvisZUpFrame);
      tempPosition2d.changeFrame(worldFrame);
      pelvisPositionError.sub(tempPosition2d);

      tempError2d.setIncludingFrame(pelvisPositionError);
      tempError2d.scale(controlDT);
      pelvisPositionCumulatedError.add(tempError2d);

      double cumulativeErrorMagnitude = pelvisPositionCumulatedError.length();
      if (cumulativeErrorMagnitude > maximumIntegralError.getDoubleValue())
      {
         pelvisPositionCumulatedError.scale(maximumIntegralError.getDoubleValue() / cumulativeErrorMagnitude);
      }

      proportionalTerm.set(pelvisPositionError);
      proportionalTerm.scale(proportionalGain.getDoubleValue());

      integralTerm.set(pelvisPositionCumulatedError);
      integralTerm.scale(integralGain.getDoubleValue());

      desiredICPOffset.set(proportionalTerm);
      desiredICPOffset.add(integralTerm);
   }

   private final ConvexPolygonScaler convexPolygonShrinker = new ConvexPolygonScaler();
   private final FrameConvexPolygon2D safeSupportPolygonToConstrainICPOffset = new FrameConvexPolygon2D();

   private final FramePoint2D originalICPToModify = new FramePoint2D();

   public void addICPOffset(FramePoint2D desiredICPToModify, FramePoint2D desiredCoMToModify, FramePoint2D desiredCoPToModify)
   {
      desiredICPToModify.changeFrame(supportPolygon.getReferenceFrame());
      desiredCoPToModify.changeFrame(supportPolygon.getReferenceFrame());
      desiredCoMToModify.changeFrame(supportPolygon.getReferenceFrame());

      originalICPToModify.setIncludingFrame(desiredICPToModify);

      if (!isEnabled.getBooleanValue() || (!isRunning.getBooleanValue() && !manualMode.getValue()))
      {
         desiredICPOffset.setToZero();
         icpOffsetForFreezing.setToZero();
         desiredICPToModify.changeFrame(worldFrame);
         desiredCoMToModify.changeFrame(worldFrame);
         desiredCoPToModify.changeFrame(worldFrame);
         return;
      }

      if (manualMode.getValue())
      {
         tempICPOffset.setIncludingFrame(supportFrame, userOffset.getX(), userOffset.getY());
      }
      else
      {
         tempICPOffset.setIncludingFrame(desiredICPOffset);
         tempICPOffset.changeFrame(supportFrame);
      }

      if (isFrozen.getBooleanValue())
      {
         desiredICPOffset.setMatchingFrame(icpOffsetForFreezing);
         desiredICPToModify.changeFrame(icpOffsetForFreezing.getReferenceFrame());
         desiredCoPToModify.changeFrame(icpOffsetForFreezing.getReferenceFrame());
         desiredCoMToModify.changeFrame(icpOffsetForFreezing.getReferenceFrame());
         desiredICPToModify.add(icpOffsetForFreezing);
         desiredCoPToModify.add(icpOffsetForFreezing);
         desiredCoMToModify.add(icpOffsetForFreezing);
      }

      else
      {
         desiredICPToModify.add(tempICPOffset);
         desiredCoPToModify.add(tempICPOffset);
         desiredCoMToModify.add(tempICPOffset);

         convexPolygonShrinker.scaleConvexPolygon(supportPolygon, supportPolygonSafeMargin.getDoubleValue(), safeSupportPolygonToConstrainICPOffset);
         safeSupportPolygonToConstrainICPOffset.orthogonalProjection(desiredICPToModify);
         safeSupportPolygonToConstrainICPOffset.orthogonalProjection(desiredCoPToModify);

         icpOffsetForFreezing.setIncludingFrame(desiredICPToModify);
         icpOffsetForFreezing.sub(originalICPToModify);
      }

      desiredICPToModify.changeFrame(worldFrame);
      desiredCoMToModify.changeFrame(worldFrame);
      desiredCoPToModify.changeFrame(worldFrame);
   }

   public void disable()
   {
      isEnabled.set(false);
      isRunning.set(false);
      isFrozen.set(false);
      isTrajectoryStopped.set(false);

      pelvisPositionError.setToZero();
      pelvisPositionCumulatedError.setToZero();

      proportionalTerm.setToZero();
      integralTerm.setToZero();

      desiredICPOffset.setToZero();
      streamTimestampOffset.setToNaN();
      streamTimestampSource.setToNaN();
   }

   public void enable()
   {
      if (isEnabled.getBooleanValue())
         return;
      isEnabled.set(true);
      isFrozen.set(false);
      isTrajectoryStopped.set(false);
      initialize();
   }

   public void freeze()
   {
      isFrozen.set(true);
      streamTimestampOffset.setToNaN();
      streamTimestampSource.setToNaN();
   }

   private void initialize()
   {
      initialPelvisPositionTime.set(yoTime.getDoubleValue());
      tempPosition.setToZero(pelvisZUpFrame);
      tempPosition.changeFrame(worldFrame);
      tempVelocity.setToZero(worldFrame);
      positionTrajectoryGenerator.clear(worldFrame);
      positionTrajectoryGenerator.appendWaypoint(0.0, tempPosition, tempVelocity);
      positionTrajectoryGenerator.initialize();
      isTrajectoryStopped.set(false);
      streamTimestampOffset.setToNaN();
      streamTimestampSource.setToNaN();
   }

   public TaskspaceTrajectoryStatusMessage pollStatusToReport()
   {
      return statusHelper.pollStatusMessage(desiredPelvisPosition, currentPelvisPosition);
   }
}
