package us.ihmc.avatar.sensors.realsense;

import controller_msgs.msg.dds.RobotConfigurationData;
import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros.RobotROSClockCalculator;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;

public class DelayedRobotConfigurationDataBuffer
{
   public static final double INITIAL_DELAY_OFFSET = 0.07; // TODO: Put in a stored property set

   private final MutableDouble delayOffset = new MutableDouble(INITIAL_DELAY_OFFSET);

   private double selectedTimestamp = 0.0;

   private boolean enabled = false;
   private final RobotROSClockCalculator rosClockCalculator;

   private IHMCROS2Callback<?> robotConfigurationDataSubscriber;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;
   private final ROS2NodeInterface ros2Node;
   private final DRCRobotModel robotModel;

   public DelayedRobotConfigurationDataBuffer(ROS2NodeInterface ros2Node, DRCRobotModel robotModel)
   {
      this(ros2Node, robotModel, INITIAL_DELAY_OFFSET);
   }

   public DelayedRobotConfigurationDataBuffer(ROS2NodeInterface ros2Node, DRCRobotModel robotModel, double delayOffset)
   {
      this.ros2Node = ros2Node;
      this.robotModel = robotModel;
      this.delayOffset.setValue(delayOffset);
      rosClockCalculator = robotModel.getROSClockCalculator();

      robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
   }

   private void acceptRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      // LogTools.info("Recieved robot configuration data w/ timestamp: {}", data.getMonotonicTime());
      robotConfigurationDataBuffer.update(robotConfigurationData);
   }

   public void setEnabled(boolean enabled)
   {
      if (this.enabled != enabled)
      {
         if (enabled)
         {
            robotConfigurationDataSubscriber = ROS2Tools.createCallbackSubscription2(ros2Node,
                                                                                     ROS2Tools.getRobotConfigurationDataTopic(robotModel.getSimpleRobotName()),
                                                                                     this::acceptRobotConfigurationData);
         }
         else
         {
            robotConfigurationDataSubscriber.destroy();
            robotConfigurationDataSubscriber = null;
         }
      }

      this.enabled = enabled;
   }

   public double updateFullRobotModel(long timestamp, FullHumanoidRobotModel fullRobotModelToUpdate)
   {
      if (enabled)
      {

         double seconds = delayOffset.getValue();
         //      LogTools.info("Latest delay: {}", seconds);
         long offsetTimestamp = timestamp - Conversions.secondsToNanoseconds(seconds);

         if (!rosClockCalculator.offsetIsDetermined())
         {
            selectedTimestamp = Double.NaN;
            return selectedTimestamp;
         }

         long controllerTime = rosClockCalculator.computeRobotMonotonicTime(offsetTimestamp);
         if (controllerTime == -1L)
         {
            selectedTimestamp = Double.NaN;
            return selectedTimestamp;
         }

         long newestTimestamp = robotConfigurationDataBuffer.getNewestTimestamp();
         if (newestTimestamp == -1L)
         {
            selectedTimestamp = Double.NaN;
            return selectedTimestamp;
         }

         selectedTimestamp = robotConfigurationDataBuffer.updateFullRobotModel(false, controllerTime, fullRobotModelToUpdate, null);
      }
      else
      {
         selectedTimestamp = Double.NaN;
      }

      return selectedTimestamp;
   }
}
