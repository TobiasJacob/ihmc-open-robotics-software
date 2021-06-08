package us.ihmc.avatar.sensors.realsense;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudMessageTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.Timer;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;

public class RealsensePointCloudROS1Bridge extends AbstractRosTopicSubscriber<sensor_msgs.PointCloud2>
{
   private static final boolean THROTTLE = false;

   private static final int MAX_POINTS = 5000;
   private static final double MIN_PUBLISH_PERIOD = UnitConversions.hertzToSeconds(10.0);

   private final boolean throttle;

   private final AtomicReference<sensor_msgs.PointCloud2> messageToProcess = new AtomicReference<>();
   private final IHMCROS2Publisher<StereoVisionPointCloudMessage> publisher;
   private final FramePose3D tempSensorFramePose = new FramePose3D();
   private final Timer throttleTimer = new Timer();
   private final ResettableExceptionHandlingExecutorService executor;

   private final DelayedReferenceFramesBuffer referenceFramesBuffer;
   private final Function<HumanoidReferenceFrames, ReferenceFrame> sensorFrameProvider;

   private final RosNodeInterface ros1Node;

   public RealsensePointCloudROS1Bridge(RosMainNode ros1Node,
                                        ROS2Node ros2Node,
                                        String ros1InputTopic,
                                        ROS2Topic<StereoVisionPointCloudMessage> ros2OutputTopic,
                                        ResettableExceptionHandlingExecutorService executor,
                                        DelayedReferenceFramesBuffer referenceFramesBuffer,
                                        Function<HumanoidReferenceFrames, ReferenceFrame> sensorFrameProvider)
   {
      this(ros1Node, ros2Node, ros1InputTopic, ros2OutputTopic, executor, referenceFramesBuffer, sensorFrameProvider, THROTTLE);
   }
   public RealsensePointCloudROS1Bridge(RosMainNode ros1Node,
                                        ROS2Node ros2Node,
                                        String ros1InputTopic,
                                        ROS2Topic<StereoVisionPointCloudMessage> ros2OutputTopic,
                                        ResettableExceptionHandlingExecutorService executor,
                                        DelayedReferenceFramesBuffer referenceFramesBuffer,
                                        Function<HumanoidReferenceFrames, ReferenceFrame> sensorFrameProvider,
                                        boolean throttle)
   {
      super(sensor_msgs.PointCloud2._TYPE);

      this.ros1Node = ros1Node;
      this.sensorFrameProvider = sensorFrameProvider;
      this.referenceFramesBuffer = referenceFramesBuffer;
      this.executor = executor;
      this.throttle = throttle;

      LogTools.info("Subscribing ROS 1: {}", ros1InputTopic);
      ros1Node.attachSubscriber(ros1InputTopic, this);

      LogTools.info("Publishing ROS 2: {}", ros2OutputTopic.getName());
      publisher = ROS2Tools.createPublisher(ros2Node, ros2OutputTopic, ROS2QosProfile.DEFAULT());
   }

   @Override
   public void onNewMessage(sensor_msgs.PointCloud2 ros1PointCloud)
   {
      messageToProcess.set(ros1PointCloud);
      if (throttle)
      {
         executor.execute(this::waitThenAct);
      }
      else
      {
         executor.execute(this::compute);
      }
   }

   private void waitThenAct()
   {
      if (messageToProcess.get() == null)
         return;

      throttleTimer.sleepUntilExpiration(MIN_PUBLISH_PERIOD);
      throttleTimer.reset();

      compute();
   }

   private void compute()
   {
      sensor_msgs.PointCloud2 ros1PointCloud = messageToProcess.getAndSet(null);
      if (ros1PointCloud == null)
         return;

      try
      {
         boolean hasColors = true;
         PointCloudData pointCloudData = new PointCloudData(ros1PointCloud, MAX_POINTS, hasColors);

         pointCloudData.flipToZUp();


         long timestamp = ros1PointCloud.getHeader().getStamp().totalNsecs();
         referenceFramesBuffer.computeReferenceFrames(timestamp);
         ReferenceFrame sensorFrame = sensorFrameProvider.apply(referenceFramesBuffer.getReferenceFrames());
         pointCloudData.applyTransform(sensorFrame.getTransformToWorldFrame());

         ArrayList<Point3D> pointCloud = new ArrayList<>();
         for (int i = 0; i < pointCloudData.getNumberOfPoints(); i++)
         {
            pointCloud.add(new Point3D(pointCloudData.getPointCloud()[i]));
         }

         tempSensorFramePose.setToZero(sensorFrame);
         tempSensorFramePose.changeFrame(ReferenceFrame.getWorldFrame());
         StereoVisionPointCloudMessage message = PointCloudMessageTools.toStereoVisionPointCloudMessage(pointCloud, tempSensorFramePose);
//         LogTools.info("Publishing point cloud of size {}", message.getNumberOfPoints());
         publisher.publish(message);
      }
      catch (Exception e)
      {
         LogTools.error(e.getMessage());
         e.printStackTrace();
      }
   }
}
