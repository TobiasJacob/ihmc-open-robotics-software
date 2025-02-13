package us.ihmc.atlas.behaviors;

import controller_msgs.msg.dds.AbortWalkingMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import org.junit.jupiter.api.*;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.dynamicsSimulation.HumanoidDynamicsSimulation;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.footstepPlanner.RemoteFootstepPlannerInterface;
import us.ihmc.behaviors.tools.footstepPlanner.RemoteFootstepPlannerResult;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AbortWalkingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2TopicNameTools;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.commons.thread.TypedNotification;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

@Tag("humanoid-behaviors")
public class AtlasFootstepPlanBehaviorTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private AtlasBehaviorTestYoVariables variables;
   private GoalOrientedTestConductor conductor;
   private AtlasRobotModel robotModel;
   private IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;
   private IHMCROS2Publisher<AbortWalkingMessage> abortPublisher;
   private ROS2Node ros2Node;

   double lastX = 0.0;
   double lastY = 0.0;
   double lastYaw = 0.0;
   private RemoteFootstepPlannerInterface remoteFootstepPlannerInterface;

   @Disabled
   @Test
   public void testExecuteFootstepPlan() throws IOException
   {
      FootstepPlanningToolboxOutputStatus output = setupForFootstepTest();

      footstepDataListPublisher.publish(output.getFootstepDataList());

      AtlasTestScripts.awaitSteps(conductor, variables, output.getFootstepDataList().getFootstepDataList().size(), 6.0);

      AtlasTestScripts.awaitDoubleSupportReachedAndHeld(conductor, variables, 3.0, 6.0);
   }

   @Disabled
   @Test
   public void testWalkNegativePiToPi() throws IOException
   {
      FootstepPlanningModuleLauncher.createModule(robotModel, PubSubImplementation.INTRAPROCESS);

      ros2Node = new ROS2Node(PubSubImplementation.INTRAPROCESS, getClass().getSimpleName());

      footstepDataListPublisher = ROS2Tools
            .createPublisherTypeNamed(ros2Node, ROS2TopicNameTools.newMessageInstance(FootstepDataListCommand.class).getMessageClass(),
                                      ROS2Tools.getControllerInputTopic(robotModel.getSimpleRobotName()));

      abortPublisher = ROS2Tools
            .createPublisherTypeNamed(ros2Node, ROS2TopicNameTools.newMessageInstance(AbortWalkingCommand.class).getMessageClass(),
                                      ROS2Tools.getControllerInputTopic(robotModel.getSimpleRobotName()));

      ROS2SyncedRobotModel syncedRobotModel = new ROS2SyncedRobotModel(robotModel, ros2Node);
      remoteFootstepPlannerInterface = new RemoteFootstepPlannerInterface(ros2Node, robotModel, null);

      AtlasTestScripts.awaitDuration(conductor, variables, 0.25);  // allows to update frames

      planSteps(0.0, 0.0, 0.5);  // 0.24 s
      planSteps(0.0, 0.0, -0.5); // 0.38 s
      planSteps(0.0, 0.0, 0.5);  // 0.64 s
      planSteps(0.0, 0.0, -0.5); // 0.25 s

      planSteps(0.0, 0.0, -0.5); // 0.008 s
      planSteps(0.0, 0.0, -1.0); // 0.028 s
      planSteps(0.0, 0.0, -3.1); // 8.498 s
      planSteps(0.0, 0.0, 3.1);  // timeout > 30s

      planSteps(0.0, 0.0, 0.5); // 0.24 s
      planSteps(0.0, 0.0, 1.0); // 0.14 s
      planSteps(0.0, 0.0, 3.1); // timeout
      planSteps(0.0, 0.0, -3.1);

      AtlasTestScripts.awaitDoubleSupportReachedAndHeld(conductor, variables, 3.0, 6.0);
   }

   private void planSteps(double x, double y, double yaw)
   {
      FramePose3D startPose = new FramePose3D();
      startPose.set(lastX, lastY, 0.0, lastYaw, 0.0, 0.0);
      LogTools.debug("StartPose = {}", startPose);

      FramePose3D goalPose = new FramePose3D();
      goalPose.set(x, y, 0.0, yaw, 0.0, 0.0);

      LogTools.info("Planning from {}, {}, yaw: {}", lastX, lastY, lastYaw);
      LogTools.info("to {}, {}, yaw: {}", x, y, yaw);

      TypedNotification<RemoteFootstepPlannerResult> resultNotification = remoteFootstepPlannerInterface.requestPlan(startPose, goalPose);

      RemoteFootstepPlannerResult result = resultNotification.blockingPoll();

      LogTools.info("Received footstep planning result: {}", FootstepPlanningResult.fromByte(result.getMessage().getFootstepPlanningResult()));
      LogTools.info("Received footstep plan took: {} s", result.getMessage().getPlannerTimings().getTotalElapsedSeconds());
      LogTools.info("Received footstep planning status: {}", result);

      assertTrue(result.isValidForExecution(), "Solution failed");

      footstepDataListPublisher.publish(result.getMessage().getFootstepDataList());

      AtlasTestScripts.awaitSteps(conductor, variables, result.getMessage().getFootstepDataList().getFootstepDataList().size(), 6.0);

      lastX = x;
      lastY = y;
      lastYaw = yaw;
   }

   @Disabled
   @Test
   public void testStopWalking() throws IOException
   {
      FootstepPlanningToolboxOutputStatus output = setupForFootstepTest();

      footstepDataListPublisher = ROS2Tools
            .createPublisherTypeNamed(ros2Node, ROS2TopicNameTools.newMessageInstance(FootstepDataListCommand.class).getMessageClass(),
                                      ROS2Tools.getControllerInputTopic(robotModel.getSimpleRobotName()));

      footstepDataListPublisher.publish(output.getFootstepDataList());

      AtlasTestScripts.awaitSteps(conductor, variables, output.getFootstepDataList().getFootstepDataList().size() / 2, 6.0);

      abortPublisher.publish(new AbortWalkingMessage());   // stop

      AtlasTestScripts.awaitDoubleSupportReachedAndHeld(conductor, variables, 3.0, 0.5);
   }

   private FootstepPlanningToolboxOutputStatus setupForFootstepTest() throws IOException
   {
      FootstepPlanningModuleLauncher.createModule(robotModel, PubSubImplementation.INTRAPROCESS);

      ros2Node = new ROS2Node(PubSubImplementation.INTRAPROCESS, getClass().getSimpleName());

      footstepDataListPublisher = ROS2Tools
            .createPublisherTypeNamed(ros2Node, ROS2TopicNameTools.newMessageInstance(FootstepDataListCommand.class).getMessageClass(),
                                      ROS2Tools.getControllerInputTopic(robotModel.getSimpleRobotName()));

      abortPublisher = ROS2Tools
            .createPublisherTypeNamed(ros2Node, ROS2TopicNameTools.newMessageInstance(AbortWalkingCommand.class).getMessageClass(),
                                      ROS2Tools.getControllerInputTopic(robotModel.getSimpleRobotName()));

      ROS2SyncedRobotModel syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);
      RemoteFootstepPlannerInterface remoteFootstepPlannerInterface = new RemoteFootstepPlannerInterface(ros2Node, robotModel, null);

      AtlasTestScripts.awaitDuration(conductor, variables, 0.25);  // allows to update frames

      syncedRobot.update();
      FramePose3D midFeetZUpPose = new FramePose3D(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
      LogTools.debug("MidFeetZUp = {}", midFeetZUpPose);

      FramePose3D currentGoalWaypoint = new FramePose3D();
      currentGoalWaypoint.prependTranslation(2.0, 0.0, 0.0);
      RemoteFootstepPlannerResult output = remoteFootstepPlannerInterface.requestPlan(midFeetZUpPose, currentGoalWaypoint).blockingPoll();

      LogTools.info("Received footstep planning status: {}", output);

      assertTrue(output.isValidForExecution(), "Solution failed");
      return output.getMessage();
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      conductor.concludeTesting();
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @BeforeEach
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      SimulationConstructionSet scs = HumanoidDynamicsSimulation.createForAutomatedTest(robotModel, new FlatGroundEnvironment()).getSimulationConstructionSet();
      variables = new AtlasBehaviorTestYoVariables(scs);
      conductor = new GoalOrientedTestConductor(scs, simulationTestingParameters);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      AtlasTestScripts.awaitStandUp(conductor, variables);
   }

   @AfterAll
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(AtlasFootstepPlanBehaviorTest.class + " after class.");
   }
}
