package us.ihmc.avatar.behaviorTests;

import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ObjectWeightBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class DRCObjectWeightBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final double epsilon = 10e-8;
   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   @BeforeEach
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DefaultCommonAvatarEnvironment testEnvironment = new DefaultCommonAvatarEnvironment();
      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, getSimpleRobotName(), DRCObstacleCourseStartingLocation.DEFAULT,
                                                        simulationTestingParameters, getRobotModel());
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.destroySimulation();
         drcBehaviorTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testConstructorAndSetInput()
   {
      ObjectWeightBehavior behavior = new ObjectWeightBehavior(drcBehaviorTestHelper.getRobotName(), drcBehaviorTestHelper.getROS2Node());
      behavior.setInput(HumanoidMessageTools.createObjectWeightPacket(RobotSide.LEFT, 0.0));
      assertTrue(behavior.hasInputBeenSet());
   }

   @Disabled("Needs to be reimplemented")
   @Test
   public void testSettingWeight() throws SimulationExceededMaximumTimeException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      ObjectWeightBehavior objectWeightBehavior = new ObjectWeightBehavior(drcBehaviorTestHelper.getRobotName(), drcBehaviorTestHelper.getROS2Node());
      YoDouble rightMass = (YoDouble) drcBehaviorTestHelper.getSimulationConstructionSet().findVariable("rightTool", "rightToolObjectMass");
      YoDouble leftMass = (YoDouble) drcBehaviorTestHelper.getSimulationConstructionSet().findVariable("leftTool", "leftToolObjectMass");

      double weightLeft = 1.5;
      objectWeightBehavior.initialize();
      objectWeightBehavior.setInput(HumanoidMessageTools.createObjectWeightPacket(RobotSide.LEFT, weightLeft));
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(objectWeightBehavior);
      assertTrue(success);
      assertTrue(MathTools.epsilonEquals(leftMass.getDoubleValue(), weightLeft, epsilon));
      assertTrue(MathTools.epsilonEquals(rightMass.getDoubleValue(), 0.0, epsilon));

      double weightRight = 0.8;
      objectWeightBehavior.initialize();
      objectWeightBehavior.setInput(HumanoidMessageTools.createObjectWeightPacket(RobotSide.RIGHT, weightRight));
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(objectWeightBehavior);
      assertTrue(success);
      assertTrue(MathTools.epsilonEquals(leftMass.getDoubleValue(), weightLeft, epsilon));
      assertTrue(MathTools.epsilonEquals(rightMass.getDoubleValue(), weightRight, epsilon));
   }
}
