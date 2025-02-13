package us.ihmc.valkyrie.pushRecovery;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.DRCPushRecoveryWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyriePushRecoveryWalkingTest extends DRCPushRecoveryWalkingTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testPushLeftEarlySwing() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(1500.0);
      super.testPushLeftEarlySwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testPushLeftInitialTransferState() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(1000.0);
      super.testPushLeftInitialTransferState();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushRightInitialTransferState() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(1500.0);
      super.testPushRightInitialTransferState();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushRightLateSwing() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(1500.0);
      super.testPushRightLateSwing();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushRightThenLeftMidSwing() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(1500.0);
      super.testPushRightThenLeftMidSwing();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushRightTransferState() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(1300.0);
      super.testPushRightTransferState();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushTowardsTheBack() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(1500.0);
      super.testPushTowardsTheBack();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushTowardsTheFront() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(1500.0);
      super.testPushTowardsTheFront();
   }
}