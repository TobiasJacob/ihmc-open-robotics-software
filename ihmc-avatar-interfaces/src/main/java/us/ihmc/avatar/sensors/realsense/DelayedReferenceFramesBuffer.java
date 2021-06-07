package us.ihmc.avatar.sensors.realsense;

import controller_msgs.msg.dds.RobotConfigurationData;
import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros.RobotROSClockCalculator;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.utilities.ros.RosNodeInterface;

public class DelayedReferenceFramesBuffer
{
   public static final double INITIAL_DELAY_OFFSET = 0.07;

   private final MutableDouble delayOffset = new MutableDouble(INITIAL_DELAY_OFFSET);

   private long selectedTimestamp = -1L;
   private long currentTimestampOffset = -1L;

   private boolean enabled = false;
   private final RobotROSClockCalculator rosClockCalculator;

   private IHMCROS2Callback<?> robotConfigurationDataSubscriber;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;
   private final ROS2NodeInterface ros2Node;
   private final DRCRobotModel robotModel;
   private final HumanoidReferenceFrames referenceFrames;
   private final FullHumanoidRobotModel fullRobotModel;


   public DelayedReferenceFramesBuffer(ROS2NodeInterface ros2Node, DRCRobotModel robotModel)
   {
      this(ros2Node, robotModel, INITIAL_DELAY_OFFSET);
   }

   public DelayedReferenceFramesBuffer(ROS2NodeInterface ros2Node, DRCRobotModel robotModel, double delayOffset)
   {
      this.ros2Node = ros2Node;
      this.robotModel = robotModel;
      this.delayOffset.setValue(delayOffset);
      rosClockCalculator = robotModel.getROSClockCalculator();

      robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();

      fullRobotModel = robotModel.createFullRobotModel();
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel, robotModel.getSensorInformation());
   }

   public void setDelayOffset(double delayOffset)
   {
      this.delayOffset.setValue(delayOffset);
   }

   public void setEnabled(boolean enabled)
   {
      if (this.enabled != enabled)
      {
         if (enabled)
         {
            robotConfigurationDataSubscriber = ROS2Tools.createCallbackSubscription2(ros2Node,
                                                                                     ROS2Tools.getRobotConfigurationDataTopic(robotModel.getSimpleRobotName()),
                                                                                     robotConfigurationDataBuffer::update);
         }
         else
         {
            robotConfigurationDataSubscriber.destroy();
            robotConfigurationDataSubscriber = null;
         }
      }

      this.enabled = enabled;
   }


   public void subscribe(RosNodeInterface ros1Node)
   {
      rosClockCalculator.subscribeToROS1Topics(ros1Node);
   }

   public void unsubscribe(RosNodeInterface ros1Node)
   {
      rosClockCalculator.unsubscribeFromROS1Topics(ros1Node);
   }

   public long computeReferenceFrames(long timestamp)
   {
      if (enabled)
      {
         double seconds = delayOffset.getValue();
         //      LogTools.info("Latest delay: {}", seconds);
         long offsetTimestamp = timestamp - Conversions.secondsToNanoseconds(seconds);

         if (!rosClockCalculator.offsetIsDetermined())
         {
            selectedTimestamp = -1L;
            currentTimestampOffset = -1L;
            return selectedTimestamp;
         }

         long controllerTime = rosClockCalculator.computeRobotMonotonicTime(offsetTimestamp);
         if (controllerTime == -1L)
         {
            selectedTimestamp = -1L;
            currentTimestampOffset = -1L;
            return selectedTimestamp;
         }

         long newestTimestamp = robotConfigurationDataBuffer.getNewestTimestamp();
         if (newestTimestamp == -1L)
         {
            selectedTimestamp = -1L;
            currentTimestampOffset = -1L;
            return selectedTimestamp;
         }

         selectedTimestamp = robotConfigurationDataBuffer.updateFullRobotModel(false, controllerTime, fullRobotModel, null);
         if (selectedTimestamp != -1L)
            currentTimestampOffset = rosClockCalculator.getCurrentTimestampOffset();

         try
         {
            referenceFrames.updateFrames();
         }
         catch (NotARotationMatrixException e)
         {
            LogTools.error(e.getMessage());
         }
      }
      else
      {
         selectedTimestamp = -1L;
         currentTimestampOffset = -1L;
      }

      return selectedTimestamp;
   }

   public long getSelectedTimestamp()
   {
      return selectedTimestamp;
   }

   public long getCurrentTimestampOffset()
   {
      return currentTimestampOffset;
   }

   public HumanoidReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }
}
