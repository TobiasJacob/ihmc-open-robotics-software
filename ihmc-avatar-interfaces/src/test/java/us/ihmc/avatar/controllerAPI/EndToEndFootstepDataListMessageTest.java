package us.ihmc.avatar.controllerAPI;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoVariable;

public abstract class EndToEndFootstepDataListMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final CommonAvatarEnvironmentInterface environment = new FlatGroundEnvironment();
   private static final DRCStartingLocation location = DRCObstacleCourseStartingLocation.DEFAULT;

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @Test
   public void testQueuing() throws SimulationExceededMaximumTimeException
   {
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.setTestEnvironment(environment);
      drcSimulationTestHelper.setStartingLocation(location);
      drcSimulationTestHelper.createSimulation("Test");
      setupCamera(drcSimulationTestHelper);
      ThreadTools.sleep(1000);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.25));

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      referenceFrames.updateFrames();
      MovingReferenceFrame midFeetFrame = referenceFrames.getMidFootZUpGroundFrame();

      double maxStepWidth = robotModel.getWalkingControllerParameters().getSteppingParameters().getMaxStepWidth();
      double minStepWidth = robotModel.getWalkingControllerParameters().getSteppingParameters().getMinStepWidth();
      double halfStepWidth = (maxStepWidth + minStepWidth) / 4.0;
      double stepLength = robotModel.getWalkingControllerParameters().getSteppingParameters().getDefaultStepLength() * 0.5;
      double nominalSwingTime = robotModel.getWalkingControllerParameters().getDefaultSwingTime();
      double nominalTransferTime = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      Random random = new Random(24384523737236643L);

      List<FootstepDataMessage> footstepList = new ArrayList<>();
      RobotSide stepSide = RobotSide.LEFT;
      int steps = 8;
      int stepsPerMessage = 3;
      double totalTime = robotModel.getWalkingControllerParameters().getDefaultFinalTransferTime();

      for (int foostepIdx = 0; foostepIdx < steps; foostepIdx++)
      {
         // walk forward
         FramePoint3D frameLocation = new FramePoint3D(midFeetFrame);
         frameLocation.setX(stepLength * (foostepIdx + 1));
         frameLocation.setY(stepSide.negateIfRightSide(halfStepWidth));
         frameLocation.changeFrame(ReferenceFrame.getWorldFrame());
         FrameQuaternion frameOrientation = new FrameQuaternion(midFeetFrame);
         frameOrientation.changeFrame(ReferenceFrame.getWorldFrame());
         FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(stepSide, frameLocation, frameOrientation);

         // between 0.75 and 1.25 times the nominal time:
         double swingTime = (1.0 + 0.5 * (random.nextDouble() + 0.5)) * nominalSwingTime;
         double transferTime = (1.0 + 0.5 * (random.nextDouble() + 0.5)) * nominalTransferTime;
         footstep.setSwingDuration(swingTime);
         footstep.setTransferDuration(transferTime);

         footstepList.add(footstep);
         totalTime += swingTime + transferTime;
         stepSide = stepSide.getOppositeSide();
      }

      List<FootstepDataListMessage> messages = new ArrayList<>();
      int stepsPackedInMessage = 0;
      while (stepsPackedInMessage < footstepList.size())
      {
         FootstepDataListMessage message = new FootstepDataListMessage();
         for (int footstepIdx = 0; footstepIdx < stepsPerMessage; footstepIdx++)
         {
            if (stepsPackedInMessage == footstepList.size())
            {
               break;
            }

            message.getFootstepDataList().add().set(footstepList.get(stepsPackedInMessage));
            stepsPackedInMessage++;
         }
         message.getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
         message.getQueueingProperties().setPreviousMessageId(FootstepDataListMessage.VALID_MESSAGE_DEFAULT_ID);
         messages.add(message);
      }
      FootstepDataListMessage r = messages.get(0);
      r.getQueueingProperties().setExecutionMode(ExecutionMode.OVERRIDE.toByte());
      r.getQueueingProperties().setPreviousMessageId(FootstepDataListMessage.VALID_MESSAGE_DEFAULT_ID);

      YoVariable numberOfStepsInController = drcSimulationTestHelper.getSimulationConstructionSet().findVariable(WalkingMessageHandler.class.getSimpleName(), "currentNumberOfFootsteps");
      int expectedNumberOfSteps = 0;

      double timeBetweenSendingMessages = nominalTransferTime / messages.size();
      double timeUntilDone = totalTime;
      for (int messageIdx = 0; messageIdx < messages.size(); messageIdx++)
      {
         drcSimulationTestHelper.publishToController(messages.get(messageIdx));
         expectedNumberOfSteps += messages.get(messageIdx).getFootstepDataList().size();
         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeBetweenSendingMessages));
         assertEquals(expectedNumberOfSteps, (int) numberOfStepsInController.getValueAsLongBits());
         timeUntilDone -= timeBetweenSendingMessages;
      }

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeUntilDone + 0.25));
      assertEquals(0, (int) numberOfStepsInController.getValueAsLongBits());
   }

   protected void testMessageIsHandled(FootstepDataListMessage messageInMidFeetZUp) throws SimulationExceededMaximumTimeException
   {
      DRCRobotModel robotModel = getRobotModel();
      CommonAvatarEnvironmentInterface environment = new FlatGroundEnvironment();
      DRCStartingLocation location = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, environment);
      drcSimulationTestHelper.setStartingLocation(location);
      drcSimulationTestHelper.createSimulation("MessageTest");
      ThreadTools.sleep(1000);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.25));

      MovingReferenceFrame messageFrame = drcSimulationTestHelper.getReferenceFrames().getMidFeetZUpFrame();
      transformMessageToWorld(messageFrame, messageInMidFeetZUp);

      drcSimulationTestHelper.publishToController(messageInMidFeetZUp);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.25));

      int steps = (int) drcSimulationTestHelper.getYoVariable("currentNumberOfFootsteps").getValueAsDouble();
      assertEquals(messageInMidFeetZUp.getFootstepDataList().size(), steps);
   }

   private static void transformMessageToWorld(ReferenceFrame messageFrame, FootstepDataListMessage message)
   {
      int steps = message.getFootstepDataList().size();
      FramePose3D stepPose = new FramePose3D();
      for (int i = 0; i < steps; i++)
      {
         FootstepDataMessage footstep = message.getFootstepDataList().get(i);
         stepPose.setIncludingFrame(messageFrame, footstep.getLocation(), footstep.getOrientation());
         stepPose.changeFrame(ReferenceFrame.getWorldFrame());
         footstep.getLocation().set(stepPose.getPosition());
         footstep.getOrientation().set(stepPose.getOrientation());
      }
   }

   private static void setupCamera(DRCSimulationTestHelper drcSimulationTestHelper)
   {
      OffsetAndYawRobotInitialSetup startingLocationOffset = location.getStartingLocationOffset();
      Point3D cameraFocus = new Point3D(startingLocationOffset.getAdditionalOffset());
      cameraFocus.addZ(1.0);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationYawAndZeroTranslation(startingLocationOffset.getYaw());
      Point3D cameraPosition = new Point3D(10.0, 5.0, cameraFocus.getZ() + 2.0);
      transform.transform(cameraPosition);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFocus, cameraPosition);
   }

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
