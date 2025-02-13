package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.JumpingFootControlModule.ConstraintType;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JumpingFeetManager
{
   private static final boolean USE_WORLDFRAME_SURFACE_NORMAL_WHEN_FULLY_CONSTRAINED = true;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final SideDependentList<JumpingFootControlModule> footControlModules = new SideDependentList<>();

   private final SideDependentList<ContactableFoot> feet;
   private final SideDependentList<FootSwitchInterface> footSwitches;

   public JumpingFeetManager(JumpingControllerToolbox controllerToolbox,
                             WalkingControllerParameters walkingControllerParameters,
                             PIDSE3GainsReadOnly swingFootGains,
                             YoRegistry parentRegistry)
   {
      feet = controllerToolbox.getContactableFeet();

      this.footSwitches = controllerToolbox.getFootSwitches();

      for (RobotSide robotSide : RobotSide.values)
      {
         JumpingFootControlModule footControlModule = new JumpingFootControlModule(robotSide,
                                                                                   walkingControllerParameters,
                                                                                   swingFootGains,
                                                                                   controllerToolbox,
                                                                                   registry);

         footControlModules.put(robotSide, footControlModule);
      }

      parentRegistry.addChild(registry);
   }

   public void setWeights(Vector3DReadOnly loadedFootAngularWeight,
                          Vector3DReadOnly loadedFootLinearWeight,
                          Vector3DReadOnly footAngularWeight,
                          Vector3DReadOnly footLinearWeight)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         JumpingFootControlModule footControlModule = footControlModules.get(robotSide);
         footControlModule.setWeights(loadedFootAngularWeight, loadedFootLinearWeight, footAngularWeight, footLinearWeight);
      }
   }

   public void initialize()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footControlModules.get(robotSide).initialize();
      }
   }

   public void compute()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footSwitches.get(robotSide).hasFootHitGround(); //debug
         footControlModules.get(robotSide).doControl();
      }
   }

   public void initializeSwingTrajectoryPreview(RobotSide upcomingSwingSide, FramePose3DReadOnly footstepPoseRelativeToTouchdownCoM, double swingHeight, double swingTime)
   {
      footControlModules.get(upcomingSwingSide).initializeSwingTrajectoryPreview(footstepPoseRelativeToTouchdownCoM, swingHeight, swingTime);
   }

   public void updateSwingTrajectoryPreview(RobotSide upcomingSwingSide, FramePose3DReadOnly adjustedFootstep)
   {
      footControlModules.get(upcomingSwingSide).setAdjustedFootstep(adjustedFootstep);
      footControlModules.get(upcomingSwingSide).updateSwingTrajectoryPreview();
   }

   public void adjustSwingTrajectory(RobotSide swingSide, FramePose3DReadOnly adjustedFootstep)
   {
      footControlModules.get(swingSide).setAdjustedFootstep(adjustedFootstep);
   }

   public void requestSwing(RobotSide upcomingSwingSide,
                            FramePose3DReadOnly footstepPoseRelativeToTouchdownCoM,
                            double swingHeight,
                            double swingTime)
   {
      JumpingFootControlModule footControlModule = footControlModules.get(upcomingSwingSide);
      footControlModule.setFootstep(footstepPoseRelativeToTouchdownCoM, swingHeight, swingTime);
      setContactStateForSwing(upcomingSwingSide);
   }

   public MultipleWaypointsPoseTrajectoryGenerator getSwingTrajectory(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getSwingTrajectory();
   }

   public ConstraintType getCurrentConstraintType(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getCurrentConstraintType();
   }

   public void initializeContactStatesForDoubleSupport(RobotSide transferToSide)
   {
      if (transferToSide == null)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            setFlatFootContactState(robotSide);
         }
      }
      else
      {
         if (getCurrentConstraintType(transferToSide.getOppositeSide()) == ConstraintType.SWING) // That case happens when doing 2 steps on same side
            setFlatFootContactState(transferToSide.getOppositeSide());
         setFlatFootContactState(transferToSide); // still need to determine contact state for trailing leg. This is done in doAction as soon as the previous ICP trajectory is done
      }
   }


   private final FrameVector3D footNormalContactVector = new FrameVector3D(worldFrame, 0.0, 0.0, 1.0);

   public void setFlatFootContactState(RobotSide robotSide)
   {
      if (USE_WORLDFRAME_SURFACE_NORMAL_WHEN_FULLY_CONSTRAINED)
         footNormalContactVector.setIncludingFrame(worldFrame, 0.0, 0.0, 1.0);
      else
         footNormalContactVector.setIncludingFrame(feet.get(robotSide).getSoleFrame(), 0.0, 0.0, 1.0);
      footControlModules.get(robotSide).setContactState(ConstraintType.FULL, footNormalContactVector);
   }

   public void setContactStateForSwing(RobotSide robotSide)
   {
      JumpingFootControlModule footControlModule = footControlModules.get(robotSide);
      footControlModule.setContactState(ConstraintType.SWING);
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getInverseDynamicsCommand();
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getFeedbackControlCommand();
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      for (RobotSide robotSide : RobotSide.values)
      {
         FeedbackControlCommandList template = footControlModules.get(robotSide).createFeedbackControlTemplate();
         for (int i = 0; i < template.getNumberOfCommands(); i++)
            ret.addCommand(template.getCommand(i));
      }
      return ret;
   }
}
