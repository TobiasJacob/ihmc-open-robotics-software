package us.ihmc.avatar.sensors.realsense;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;

import java.net.URI;

public class RealsenseROS1Bridge
{
   private final DelayedReferenceFramesBuffer referenceFrameBuffer;

   private final RealsenseVideoROS1Bridge d435VideoBridge;
   private final RealsenseVideoROS1Bridge l515VideoBridge;
   private final RealsensePointCloudROS1Bridge d435PointCloudBridge;
   private final RealsensePointCloudROS1Bridge l515PointCloudBridge;
   //   private final MapSensePlanarRegionROS1Bridge l515PlanarRegionBridge;

   public RealsenseROS1Bridge(DRCRobotModel robotModel)
   {
      URI masterURI = NetworkParameters.getROSURI();
      LogTools.info("Connecting to ROS 1 master URI: {}", masterURI);
      RosMainNode ros1Node = new RosMainNode(masterURI, "ImagePublisher", true);

      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "imagePublisherNode");

      referenceFrameBuffer = new DelayedReferenceFramesBuffer(ros2Node, robotModel);
      referenceFrameBuffer.subscribe(ros1Node);
      referenceFrameBuffer.setEnabled(true);

      d435VideoBridge = new RealsenseVideoROS1Bridge(ros1Node,
                                                     ros2Node,
                                                     RosTools.D435_VIDEO,
                                                     ROS2Tools.D435_VIDEO,
                                                     referenceFrameBuffer,
                                                     HumanoidReferenceFrames::getObjectDetectionCameraFrame);
      l515VideoBridge = new RealsenseVideoROS1Bridge(ros1Node,
                                                     ros2Node,
                                                     RosTools.L515_VIDEO,
                                                     ROS2Tools.L515_VIDEO,
                                                     referenceFrameBuffer,
                                                     HumanoidReferenceFrames::getSteppingCameraFrame);

      d435PointCloudBridge = new RealsensePointCloudROS1Bridge(ros1Node,
                                                               ros2Node,
                                                               RosTools.D435_POINT_CLOUD,
                                                               ROS2Tools.D435_POINT_CLOUD,
                                                               referenceFrameBuffer,
                                                               HumanoidReferenceFrames::getObjectDetectionCameraFrame);
      l515PointCloudBridge = new RealsensePointCloudROS1Bridge(ros1Node,
                                                               ros2Node,
                                                               RosTools.L515_POINT_CLOUD,
                                                               ROS2Tools.L515_POINT_CLOUD,
                                                               referenceFrameBuffer,
                                                               HumanoidReferenceFrames::getSteppingCameraFrame);
      //      l515PlanarRegionBridge = new MapSensePlanarRegionROS1Bridge(robotModel,
      //                                                                  ros1Node,
      //                                                                  ros2Node);

      ros1Node.execute();
   }
}
