package us.ihmc.avatar.sensors.realsense;

import controller_msgs.msg.dds.RobotConfigurationData;
import map_sense.RawGPUPlanarRegionList;
import org.apache.commons.lang3.mutable.MutableDouble;
import org.jfree.util.Log;
import org.ros.message.Time;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros.RobotROSClockCalculator;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.updaters.GPUPlanarRegionUpdater;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.publisher.RosPoseStampedPublisher;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.util.function.Consumer;

public class DelayFixedPlanarRegionsSubscription
{
   public static final double INITIAL_DELAY_OFFSET = 0.07;

   private final ResettableExceptionHandlingExecutorService executorService;
   private final GPUPlanarRegionUpdater gpuPlanarRegionUpdater = new GPUPlanarRegionUpdater();
   private final HumanoidReferenceFrames referenceFrames;
   private final String topic;
   private final Consumer<PlanarRegionsList> callback;
   private final FullHumanoidRobotModel fullRobotModel;
   private final RosPoseStampedPublisher sensorPosePublisher;
   private boolean posePublisherEnabled = false;

   private final DelayedRobotConfigurationDataBuffer delayedRobotConfigurationDataBuffer;
   private boolean enabled = false;
   private AbstractRosTopicSubscriber<RawGPUPlanarRegionList> subscriber;
   private double delay = 0.0;
   private RosNodeInterface ros1Node;

   public DelayFixedPlanarRegionsSubscription(ROS2NodeInterface ros2Node,
                                              DRCRobotModel robotModel,
                                              String topic,
                                              Consumer<PlanarRegionsList> callback)
   {
      this.topic = topic;
      this.callback = callback;

      delayedRobotConfigurationDataBuffer = new DelayedRobotConfigurationDataBuffer(ros2Node, robotModel, INITIAL_DELAY_OFFSET);

      ROS2Tools.createCallbackSubscription2(ros2Node, ROS2Tools.MAPSENSE_REGIONS_DELAY_OFFSET, message -> delayedRobotConfigurationDataBuffer.setDelayOffset(message.getData()));

      boolean daemon = true;
      int queueSize = 1;
      executorService = MissingThreadTools.newSingleThreadExecutor("ROS1PlanarRegionsSubscriber", daemon, queueSize);

      gpuPlanarRegionUpdater.attachROS2Tuner(ros2Node);

      fullRobotModel = robotModel.createFullRobotModel();

      sensorPosePublisher = new RosPoseStampedPublisher(false);

      referenceFrames = new HumanoidReferenceFrames(fullRobotModel, robotModel.getSensorInformation());
   }

   public void subscribe(RosNodeInterface ros1Node)
   {
      this.ros1Node = ros1Node;
      LogTools.info("Attaching Publisher for Pose.");
      this.ros1Node.attachPublisher("/atlas/sensors/chest_l515/pose",sensorPosePublisher);
      delayedRobotConfigurationDataBuffer.subscribe(ros1Node);
      subscriber = MapsenseTools.createROS1Callback(topic, ros1Node, this::acceptRawGPUPlanarRegionsList);
   }

   public void unsubscribe(RosNodeInterface ros1Node)
   {
      ros1Node.removeSubscriber(subscriber);
      delayedRobotConfigurationDataBuffer.unsubscribe(ros1Node);
   }

   private void acceptRawGPUPlanarRegionsList(RawGPUPlanarRegionList rawGPUPlanarRegionList)
   {
      if (enabled)
      {
         executorService.clearQueueAndExecute(() ->
         {
            long timestamp = rawGPUPlanarRegionList.getHeader().getStamp().totalNsecs();

            long selectedTimestamp = delayedRobotConfigurationDataBuffer.updateFullRobotModel(timestamp, fullRobotModel);
            if (selectedTimestamp != -1L)
            {
               long currentTimeInWall = ros1Node.getCurrentTime().totalNsecs();
               long selectedTimeInWall = selectedTimestamp - delayedRobotConfigurationDataBuffer.getCurrentTimestampOffset();
               delay = Conversions.nanosecondsToSeconds(currentTimeInWall - selectedTimeInWall);

               try
               {
                  referenceFrames.updateFrames();
               }
               catch (NotARotationMatrixException e)
               {
                  LogTools.error(e.getMessage());
               }

               PlanarRegionsList planarRegionsList = gpuPlanarRegionUpdater.generatePlanarRegions(rawGPUPlanarRegionList);
               try
               {
                  planarRegionsList.applyTransform(MapsenseTools.getTransformFromCameraToWorld());
                  planarRegionsList.applyTransform(referenceFrames.getSteppingCameraFrame().getTransformToWorldFrame());

                  if(posePublisherEnabled)
                  {
                     RigidBodyTransform transform = referenceFrames.getSteppingCameraFrame().getTransformToWorldFrame();
                     sensorPosePublisher.publish("chest_l515",
                                                 (Vector3D) transform.getTranslation(),
                                                 new Quaternion(transform.getRotation()),
                                                 new Time(currentTimeInWall));
                  }
               }
               catch (NotARotationMatrixException e)
               {
                  LogTools.error(e.getMessage());
               }
               callback.accept(planarRegionsList);
            }
            else
            {
               delay = Double.NaN;
            }
         });
      }
   }

   public void destroy()
   {
      executorService.destroy();
   }

   public void setEnabled(boolean enabled)
   {
      delayedRobotConfigurationDataBuffer.setEnabled(enabled);

      if (this.enabled != enabled)
      {
         if (!enabled)
            executorService.interruptAndReset();
      }

      this.enabled = enabled;
   }

   public double getDelay()
   {
      return delay;
   }

   public void setPosePublisherEnabled(boolean posePublisherEnabled)
   {
      this.posePublisherEnabled = posePublisherEnabled;
   }
}
