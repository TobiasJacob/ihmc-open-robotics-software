package us.ihmc.exampleSimulations.controllerCore.robotArmWithFixedBase;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerTemplate;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.trajectories.StraightLinePoseTrajectoryGenerator;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.exampleSimulations.controllerCore.ControllerCoreModeChangedListener;
import us.ihmc.exampleSimulations.controllerCore.RobotArmControllerCoreOptimizationSettings;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.controllers.pidGains.implementations.SymmetricYoPIDSE3Gains;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.RobotJointLimitWatcher;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class FixedBaseRobotArmController implements RobotController
{
   private static final WholeBodyControllerCoreMode defaultControlMode = WholeBodyControllerCoreMode.INVERSE_KINEMATICS;

   private static final boolean USE_PRIVILEGED_CONFIGURATION = true;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final FixedBaseRobotArm robotArm;
   private final YoDouble yoTime;
   private final CenterOfMassReferenceFrame centerOfMassFrame;

   public enum FeedbackControlType
   {
      SPATIAL, LINEAR_ANGULAR_SEPARATE
   };

   private final YoEnum<WholeBodyControllerCoreMode> controllerCoreMode = new YoEnum<>("controllerCoreMode", registry,
                                                                                                       WholeBodyControllerCoreMode.class);
   private final YoEnum<WholeBodyControllerCoreMode> controllerCoreModePrev = new YoEnum<>("controllerCoreModePrev", registry,
                                                                                       WholeBodyControllerCoreMode.class);
   private final List<ControllerCoreModeChangedListener> controllerModeListeners = new ArrayList<>();
   private final YoEnum<FeedbackControlType> feedbackControlToUse = new YoEnum<>("feedbackControlToUse", registry, FeedbackControlType.class,
                                                                                                 false);

   private final PointFeedbackControlCommand handPointCommand = new PointFeedbackControlCommand();
   private final OrientationFeedbackControlCommand handOrientationCommand = new OrientationFeedbackControlCommand();
   private final SpatialFeedbackControlCommand handSpatialCommand = new SpatialFeedbackControlCommand();
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

   private final WholeBodyControlCoreToolbox controlCoreToolbox;
   private final WholeBodyControllerCore controllerCore;

   private final YoDouble handWeight = new YoDouble("handWeight", registry);
   private final SymmetricYoPIDSE3Gains handPositionGains = new SymmetricYoPIDSE3Gains("handPosition", registry);
   private final SymmetricYoPIDSE3Gains handOrientationGains = new SymmetricYoPIDSE3Gains("handOrientation", registry);
   private final YoFramePoint3D handTargetPosition = new YoFramePoint3D("handTarget", worldFrame, registry);

   private final YoFrameYawPitchRoll handTargetOrientation = new YoFrameYawPitchRoll("handTarget", worldFrame, registry);
   private final YoBoolean goToTarget = new YoBoolean("goToTarget", registry);
   private final YoDouble trajectoryDuration = new YoDouble("handTrajectoryDuration", registry);
   private final YoDouble trajectoryStartTime = new YoDouble("handTrajectoryStartTime", registry);

   private final StraightLinePoseTrajectoryGenerator trajectory;

   private final YoBoolean controlLinearX = new YoBoolean("controlLinearX", registry);
   private final YoBoolean controlLinearY = new YoBoolean("controlLinearY", registry);
   private final YoBoolean controlLinearZ = new YoBoolean("controlLinearZ", registry);
   private final YoBoolean controlAngularX = new YoBoolean("controlAngularX", registry);
   private final YoBoolean controlAngularY = new YoBoolean("controlAngularY", registry);
   private final YoBoolean controlAngularZ = new YoBoolean("controlAngularZ", registry);

   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final RobotJointLimitWatcher robotJointLimitWatcher;

   private final YoBoolean setRandomConfiguration = new YoBoolean("setRandomConfiguration", registry);

   public FixedBaseRobotArmController(FixedBaseRobotArm robotArm, double controlDT, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.robotArm = robotArm;

      controllerCoreMode.set(defaultControlMode);
      controllerCoreModePrev.set(controllerCoreMode.getValue());

      yoTime = robotArm.getYoTime();
      double gravityZ = robotArm.getGravity();
      RigidBodyBasics hand = robotArm.getHand();
      RigidBodyBasics elevator = robotArm.getElevator();
      JointBasics[] controlledJoints = MultiBodySystemTools.collectSupportAndSubtreeJoints(elevator);
      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, elevator);

      ControllerCoreOptimizationSettings optimizationSettings = new RobotArmControllerCoreOptimizationSettings();

      controlCoreToolbox = new WholeBodyControlCoreToolbox(controlDT, gravityZ, null, controlledJoints, centerOfMassFrame,
                                                                                       optimizationSettings, yoGraphicsListRegistry, registry);

      if (USE_PRIVILEGED_CONFIGURATION)
         controlCoreToolbox.setJointPrivilegedConfigurationParameters(new JointPrivilegedConfigurationParameters());

      controlCoreToolbox.setupForInverseDynamicsSolver(new ArrayList<>());
      controlCoreToolbox.setupForInverseKinematicsSolver();

      FeedbackControlCommandList allPossibleCommands = new FeedbackControlCommandList();

      handPointCommand.set(elevator, hand);
      handOrientationCommand.set(elevator, hand);
      handSpatialCommand.set(elevator, hand);
      allPossibleCommands.addCommand(handPointCommand);
      allPossibleCommands.addCommand(handOrientationCommand);
      allPossibleCommands.addCommand(handSpatialCommand);

      JointDesiredOutputList lowLevelControllerCoreOutput = new JointDesiredOutputList(MultiBodySystemTools.filterJoints(controlledJoints, OneDoFJointBasics.class));

      controllerCore = new WholeBodyControllerCore(controlCoreToolbox, new FeedbackControllerTemplate(allPossibleCommands), lowLevelControllerCoreOutput, registry);

      yoGraphicsListRegistry.registerYoGraphic("desireds", new YoGraphicCoordinateSystem("targetFrame", handTargetPosition, handTargetOrientation, 0.15,
                                                                                         YoAppearance.Red()));

      privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationOption.AT_ZERO);
      privilegedConfigurationCommand.addJoint(robotArm.getElbowPitch(), Math.PI / 3.0);
      privilegedConfigurationCommand.setDefaultWeight(0.025);

      trajectory = new StraightLinePoseTrajectoryGenerator("handTrajectory", worldFrame, registry, true, yoGraphicsListRegistry);

      robotJointLimitWatcher = new RobotJointLimitWatcher(MultiBodySystemTools.filterJoints(controlledJoints, OneDoFJointBasics.class));
      registry.addChild(robotJointLimitWatcher.getYoRegistry());

      robotArm.setDynamic(defaultControlMode != WholeBodyControllerCoreMode.INVERSE_KINEMATICS);
      initialize();
   }

   public void registerControllerCoreModeChangedListener(ControllerCoreModeChangedListener listener)
   {
      controllerModeListeners.add(listener);
   }

   @Override
   public void initialize()
   {
      robotArm.updateIDRobot();

      for (int i = 0; i < privilegedConfigurationCommand.getNumberOfJoints(); i++)
      {
         OneDoFJointBasics joint = privilegedConfigurationCommand.getJoint(i);
         privilegedConfigurationCommand.setOneDoFJoint(i, joint.getQ());
      }

      handWeight.set(1.0);
      updateGains();

      FramePoint3D initialPosition = new FramePoint3D(robotArm.getHandControlFrame());
      initialPosition.changeFrame(worldFrame);
      FrameQuaternion initialOrientation = new FrameQuaternion(robotArm.getHandControlFrame());
      initialOrientation.changeFrame(worldFrame);

      handTargetPosition.setMatchingFrame(initialPosition);
      handTargetOrientation.setMatchingFrame(initialOrientation);

      trajectoryDuration.set(0.5);
      trajectory.setInitialPose(initialPosition, initialOrientation);
      trajectory.setFinalPose(initialPosition, initialOrientation);
      trajectory.setTrajectoryTime(trajectoryDuration.getDoubleValue());

      controlLinearX.set(true);
      controlLinearY.set(true);
      controlLinearZ.set(true);
      controlAngularX.set(true);
      controlAngularY.set(true);
      controlAngularZ.set(true);
      trajectory.showVisualization();
   }

   private void updateGains()
   {
      if (controllerCoreMode.getValue() == WholeBodyControllerCoreMode.INVERSE_DYNAMICS || controllerCoreMode.getValue() == WholeBodyControllerCoreMode.VIRTUAL_MODEL)
      {
         handPositionGains.setProportionalGains(100.0);
         handPositionGains.setDampingRatios(1.0);
         handOrientationGains.setProportionalGains(100.0);
         handOrientationGains.setDampingRatios(1.0);
      }
      else
      {
         handPositionGains.setProportionalGains(1200.0);
         handPositionGains.setDampingRatios(0.0);
         handPositionGains.setMaxFeedbackAndFeedbackRate(1500.0, Double.POSITIVE_INFINITY);

         handOrientationGains.setProportionalGains(1200.0);
         handOrientationGains.setDampingRatios(0.0);
         handOrientationGains.setMaxFeedbackAndFeedbackRate(1500.0, Double.POSITIVE_INFINITY);
      }
   }

   private final FramePoint3D position = new FramePoint3D();
   private final FrameVector3D linearVelocity = new FrameVector3D();
   private final FrameVector3D linearAcceleration = new FrameVector3D();
   private final FrameQuaternion orientation = new FrameQuaternion();
   private final FrameVector3D angularVelocity = new FrameVector3D();
   private final FrameVector3D angularAcceleration = new FrameVector3D();

   @Override
   public void doControl()
   {
      WholeBodyControllerCoreMode controllerCoreMode = this.controllerCoreMode.getValue();
      WholeBodyControllerCoreMode controllerCoreModePrev = this.controllerCoreModePrev.getValue();
      if (controllerCoreMode == WholeBodyControllerCoreMode.VIRTUAL_MODEL || controllerCoreMode == WholeBodyControllerCoreMode.OFF)
      {
         this.controllerCoreMode.set(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
         controllerCoreMode = WholeBodyControllerCoreMode.INVERSE_DYNAMICS;
      }

      robotArm.updateControlFrameAcceleration();
      robotArm.updateIDRobot();
      centerOfMassFrame.update();

      updateTrajectory();
      updateFeedbackCommands();

      controllerCoreCommand.clear();
      controllerCoreCommand.setControllerCoreMode(controllerCoreMode);

      if (feedbackControlToUse.getEnumValue() == FeedbackControlType.SPATIAL)
      {
         controllerCoreCommand.addFeedbackControlCommand(handSpatialCommand);
      }
      else
      {
         controllerCoreCommand.addFeedbackControlCommand(handPointCommand);
         controllerCoreCommand.addFeedbackControlCommand(handOrientationCommand);
      }
      if (USE_PRIVILEGED_CONFIGURATION)
         controllerCoreCommand.addInverseKinematicsCommand(privilegedConfigurationCommand);
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();

      ControllerCoreOutput controllerCoreOutput = controllerCore.getControllerCoreOutput();
      JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder = controllerCoreOutput.getLowLevelOneDoFJointDesiredDataHolder();

      if (controllerCoreMode != controllerCoreModePrev)
      {
         final WholeBodyControllerCoreMode updateMode = controllerCoreMode;
         controllerModeListeners.forEach(listener -> listener.controllerCoreModeHasChanged(updateMode));
         updateGains();
      }

      if (controllerCoreMode == WholeBodyControllerCoreMode.INVERSE_DYNAMICS)
         robotArm.updateJointTaus(lowLevelOneDoFJointDesiredDataHolder);
      else
         robotArm.updateSCSRobotJointConfiguration(lowLevelOneDoFJointDesiredDataHolder);

      if (setRandomConfiguration.getBooleanValue())
      {
         robotArm.setRandomConfiguration();
         setRandomConfiguration.set(false);
      }

      this.controllerCoreModePrev.set(controllerCoreMode);
      robotJointLimitWatcher.doControl();
   }

   public void updateFeedbackCommands()
   {
      FramePose3D controlFramePose = new FramePose3D(robotArm.getHandControlFrame());
      controlFramePose.changeFrame(robotArm.getHand().getBodyFixedFrame());

      trajectory.getAngularData(orientation, angularVelocity, angularAcceleration);
      trajectory.getLinearData(position, linearVelocity, linearAcceleration);

      handPointCommand.setBodyFixedPointToControl(controlFramePose.getPosition());
      handPointCommand.setWeightForSolver(handWeight.getDoubleValue());
      handPointCommand.setGains(handPositionGains);
      handPointCommand.setSelectionMatrix(computeLinearSelectionMatrix());
      handPointCommand.setInverseDynamics(position, linearVelocity, linearAcceleration);
      handPointCommand.setControlMode(controllerCoreMode.getValue());

      handOrientationCommand.setWeightForSolver(handWeight.getDoubleValue());
      handOrientationCommand.setGains(handOrientationGains);
      handOrientationCommand.setSelectionMatrix(computeAngularSelectionMatrix());
      handOrientationCommand.setInverseDynamics(orientation, angularVelocity, angularAcceleration);
      handOrientationCommand.setControlMode(controllerCoreMode.getValue());

      handSpatialCommand.setControlFrameFixedInEndEffector(controlFramePose);
      handSpatialCommand.setWeightForSolver(handWeight.getDoubleValue());
      handSpatialCommand.setPositionGains(handPositionGains);
      handSpatialCommand.setOrientationGains(handOrientationGains);
      handSpatialCommand.setSelectionMatrix(computeSpatialSelectionMatrix());
//      handSpatialCommand.setInverseDynamics(orientation, position, angularVelocity, linearVelocity, angularAcceleration, linearAcceleration);
      handSpatialCommand.setInverseKinematics(orientation, position, linearVelocity, angularVelocity);
      handSpatialCommand.setControlMode(controllerCoreMode.getValue());
   }

   public void updateTrajectory()
   {
      if (goToTarget.getBooleanValue())
      {
         FramePoint3D initialPosition = new FramePoint3D(robotArm.getHandControlFrame());
         initialPosition.changeFrame(worldFrame);
         FrameQuaternion initialOrientation = new FrameQuaternion(robotArm.getHandControlFrame());
         initialOrientation.changeFrame(worldFrame);
         trajectory.setInitialPose(initialPosition, initialOrientation);
         FramePoint3D finalPosition = new FramePoint3D(handTargetPosition);
         FrameQuaternion finalOrientation = new FrameQuaternion();
         finalOrientation.setIncludingFrame(handTargetOrientation);
         trajectory.setFinalPose(finalPosition, finalOrientation);
         trajectory.setTrajectoryTime(trajectoryDuration.getDoubleValue());
         trajectory.initialize();
         trajectoryStartTime.set(yoTime.getDoubleValue());
         goToTarget.set(false);
      }

      trajectory.compute(yoTime.getDoubleValue() - trajectoryStartTime.getDoubleValue());
   }

   private SelectionMatrix3D computeLinearSelectionMatrix()
   {
      SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();

      selectionMatrix.selectXAxis(controlLinearX.getBooleanValue());
      selectionMatrix.selectYAxis(controlLinearY.getBooleanValue());
      selectionMatrix.selectZAxis(controlLinearZ.getBooleanValue());

      return selectionMatrix;
   }

   private SelectionMatrix3D computeAngularSelectionMatrix()
   {
      SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();

      selectionMatrix.selectXAxis(controlAngularX.getBooleanValue());
      selectionMatrix.selectYAxis(controlAngularY.getBooleanValue());
      selectionMatrix.selectZAxis(controlAngularZ.getBooleanValue());

      return selectionMatrix;
   }

   private SelectionMatrix6D computeSpatialSelectionMatrix()
   {
      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

      selectionMatrix.selectAngularX(controlAngularX.getBooleanValue());
      selectionMatrix.selectAngularY(controlAngularY.getBooleanValue());
      selectionMatrix.selectAngularZ(controlAngularZ.getBooleanValue());

      selectionMatrix.selectLinearX(controlLinearX.getBooleanValue());
      selectionMatrix.selectLinearY(controlLinearY.getBooleanValue());
      selectionMatrix.selectLinearZ(controlLinearZ.getBooleanValue());

      return selectionMatrix;
   }

   public void setToRandomConfiguration()
   {
      setRandomConfiguration.set(true);
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public String getDescription()
   {
      return name;
   }

   public YoFramePoint3D getHandTargetPosition()
   {
      return handTargetPosition;
   }

   public YoFrameYawPitchRoll getHandTargetOrientation()
   {
      return handTargetOrientation;
   }

   public YoBoolean getGoToTarget()
   {
      return goToTarget;
   }

   public WholeBodyControlCoreToolbox getControlCoreToolbox()
   {
      return controlCoreToolbox;
   }
}
