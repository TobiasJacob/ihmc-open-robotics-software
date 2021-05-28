package us.ihmc.commonWalkingControlModules.controlModules;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.YoSwingTrajectoryParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SwingTrajectoryCalculatorTest
{
   private static final double simDt = 0.005;
   private static final int BUFFER_SIZE = 160000;

   @Test
   public void testFromInitialConditions()
   {
      FramePoint3DReadOnly initialLeftFootPosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.6569, 0.1314, 0.0187);
      FramePoint3DReadOnly initialRightFootPosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.946, -0.07, -0.0075);
      FrameQuaternionReadOnly initialLeftFootOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame(), -0.0014, 0.0183, -0.0026, 0.9998);
      FrameQuaternionReadOnly initialRightFootOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame(), -0.0118, 0.0051, 0.0101, 0.9999);
      FrameVector3DReadOnly initialLeftFootVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0187, -0.0042, -0.0039);

      MovingReferenceFrame swingSoleFrame = getSwingSoleFrame(initialLeftFootPosition, initialLeftFootOrientation, initialLeftFootVelocity);
      ReferenceFrame stanceSoleFrame = getStanceSoleFrame(initialRightFootPosition, initialRightFootOrientation);
      ReferenceFrame stanceZUpFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(), stanceSoleFrame, "stanceZUpFrame");

      swingSoleFrame.update();
      stanceSoleFrame.update();
      stanceZUpFrame.update();

      YoRegistry registry = new YoRegistry("test");
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      double minSwingHeight = 0.09;
      double defaultSwingHeight = 0.09;
      double maxSwingHeight = 0.3;
      double customWaypointAngleThreshold = 0.8727;
      double minDistanceToStance = Double.NEGATIVE_INFINITY;
      SwingTrajectoryParameters swingTrajectoryParameters = getSwingTrajectoryParameters(-0.12, 0.6);

      YoSwingTrajectoryParameters yoSwingTrajectoryParameters = new YoSwingTrajectoryParameters("",
                                                                                                swingTrajectoryParameters,
                                                                                                false,
                                                                                                Double.POSITIVE_INFINITY,
                                                                                                Double.POSITIVE_INFINITY,
                                                                                                registry);
      SwingTrajectoryCalculator swingTrajectoryCalculator = new SwingTrajectoryCalculator("",
                                                                                          RobotSide.LEFT,
                                                                                          swingSoleFrame,
                                                                                          stanceSoleFrame,
                                                                                          stanceZUpFrame,
                                                                                          minSwingHeight,
                                                                                          maxSwingHeight,
                                                                                          defaultSwingHeight,
                                                                                          customWaypointAngleThreshold,
                                                                                          minDistanceToStance,
                                                                                          yoSwingTrajectoryParameters,
                                                                                          registry,
                                                                                          graphicsListRegistry);

      DefaultParameterReader reader = new DefaultParameterReader();
      reader.readParametersInRegistry(registry);

      double duration = 0.6;
      Footstep footstep = new Footstep();
      footstep.getFootstepPose().getPosition().set(1.1413, 0.1423, -0.0013);
      footstep.getFootstepPose().getOrientation().set(0.0, 0.0, 0.0104, 0.9999);

      swingTrajectoryCalculator.setInitialConditionsToCurrent();
      swingTrajectoryCalculator.setFootstep(footstep);
      swingTrajectoryCalculator.setSwingDuration(duration);
      swingTrajectoryCalculator.initializeTrajectoryWaypoints(true);


      visualize(registry, graphicsListRegistry, swingTrajectoryCalculator, duration);
   }

   private void visualize(YoRegistry registry, YoGraphicsListRegistry graphicsListRegistry, SwingTrajectoryCalculator swingTrajectoryCalculator, double duration)
   {
      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters(true, BUFFER_SIZE);
      Robot robot = new Robot("Dummy");

      YoDouble yoTime = new YoDouble("time", registry);
      YoFramePoint3D desiredPosition = new YoFramePoint3D("desiredPosition", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector3D desiredVelocity = new YoFrameVector3D("desiredVelocity", ReferenceFrame.getWorldFrame(), registry);
      YoGraphicPosition desiredPositionViz = new YoGraphicPosition("desiredPosition", desiredPosition, 0.02, YoAppearance.Black());

      graphicsListRegistry.registerYoGraphic("Vis", desiredPositionViz);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, scsParameters);
      scs.setDT(simDt, 1);
      scs.addYoRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.setPlaybackRealTimeRate(0.75);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      scs.setCameraFix(0.0, 0.0, 0.5);
      scs.setCameraPosition(-0.5, 0.0, 1.0);

      scs.startOnAThread();

      for (double time = 0.0; time <= duration; time += 0.005)
      {
         swingTrajectoryCalculator.doOptimizationUpdate();
         swingTrajectoryCalculator.getSwingTrajectory().compute(time);
         desiredPosition.set(swingTrajectoryCalculator.getSwingTrajectory().getPosition());
         desiredVelocity.set(swingTrajectoryCalculator.getSwingTrajectory().getVelocity());

         yoTime.set(time);
         scs.tickAndUpdate();
      }

      ThreadTools.sleepForever();
   }

   private static SwingTrajectoryParameters getSwingTrajectoryParameters(double touchdownVelocity, double touchdownAcceleration)
   {
      return new SwingTrajectoryParameters()
      {
         @Override
         public boolean doToeTouchdownIfPossible()
         {
            return false;
         }

         @Override
         public double getToeTouchdownAngle()
         {
            return 0;
         }

         @Override
         public boolean doHeelTouchdownIfPossible()
         {
            return false;
         }

         @Override
         public double getHeelTouchdownAngle()
         {
            return 0;
         }

         @Override
         public double getDesiredTouchdownHeightOffset()
         {
            return 0;
         }

         @Override
         public double getDesiredTouchdownVelocity()
         {
            return touchdownVelocity;
         }

         @Override
         public double getDesiredTouchdownAcceleration()
         {
            return touchdownAcceleration;
         }

         @Override
         public double getMinMechanicalLegLength()
         {
            return 0;
         }
      };
   }

   private static MovingReferenceFrame getSwingSoleFrame(FramePoint3DReadOnly initialPosition, FrameQuaternionReadOnly initialOrientation, FrameVector3DReadOnly linearVelocity)
   {
      return new MovingReferenceFrame("swingSoleFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
         {
            twistRelativeToParentToPack.getLinearPart().setMatchingFrame(linearVelocity);
         }

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.getTranslation().set(initialPosition);
            transformToParent.getRotation().set(initialOrientation);
         }
      };
   }

   private static ReferenceFrame getStanceSoleFrame(FramePoint3DReadOnly initialPosition, FrameQuaternionReadOnly initialOrientation)
   {
      return new ReferenceFrame("stanceSoleFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.getTranslation().set(initialPosition);
            transformToParent.getRotation().set(initialOrientation);
         }
      };
   }
}
