package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

import java.util.ArrayList;
import java.util.List;

/**
 * This command is designed to minimize the difference between a contact acceleration exerted over the course of the segment duration and some desired value.
 * The exact cost is the integral of the squared difference between the contact acceleration and a desired value.
 */
public class ForceRateTrackingCommand implements MPCCommand<ForceRateTrackingCommand>
{
   private int commandId;
   /**
    * Defines the contact planes to be used in the force minimization.
    */
   private final List<MPCContactPlane> contactPlanes = new ArrayList<>();

   /**
    * Specifies the segment corresponding to these contacts.
    */
   private int segmentNumber;
   /**
    * Time constant used in the CoM function for this command.
    */
   private double omega;
   /**
    * Weight of this minimization command in the optimizer.
    */
   private double weight;

   private double duration;

   private final FrameVector3D objectiveValue = new FrameVector3D();

   /**
    * @return command type for the MPC core.
    */
   public MPCCommandType getCommandType()
   {
      return MPCCommandType.FORCE_RATE_TRACKING;
   }

   /**
    * Resets the command
    */
   public void clear()
   {
      segmentNumber = -1;
      duration = Double.NaN;
      objectiveValue.setToNaN();
      contactPlanes.clear();
   }

   /**
    * Sets the weight for the value cost.
    */
   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   /**
    * Adds a contact whose force should be minimized
    */
   public void addContactPlaneHelper(MPCContactPlane contactPlaneHelper)
   {
      this.contactPlanes.add(contactPlaneHelper);
   }

   /**
    * Sets the segment corresponding to these contacts.
    */
   public void setSegmentNumber(int segmentNumber)
   {
      this.segmentNumber = segmentNumber;
   }

   /**
    * Sets the time constant for the motion function.
    */
   public void setOmega(double omega)
   {
      this.omega = omega;
   }

   public void setSegmentDuration(double segmentDuration)
   {
      this.duration = segmentDuration;
   }

   public void setObjectiveValue(FrameVector3DReadOnly objectiveValue)
   {
      this.objectiveValue.set(objectiveValue);
   }

   public int getSegmentNumber()
   {
      return segmentNumber;
   }

   public double getWeight()
   {
      return weight;
   }

   public double getOmega()
   {
      return omega;
   }

   public int getNumberOfContacts()
   {
      return contactPlanes.size();
   }

   public MPCContactPlane getContactPlaneHelper(int i)
   {
      return contactPlanes.get(i);
   }

   public FrameVector3DReadOnly getObjectiveValue()
   {
      return objectiveValue;
   }

   public double getSegmentDuration()
   {
      return duration;
   }

   @Override
   public void set(ForceRateTrackingCommand other)
   {
      clear();
      setCommandId(other.getCommandId());
      setSegmentNumber(other.getSegmentNumber());
      setOmega(other.getOmega());
      setWeight(other.getWeight());
      setObjectiveValue(other.getObjectiveValue());
      setSegmentDuration(other.getSegmentDuration());
      for (int i = 0; i < other.getNumberOfContacts(); i++)
         addContactPlaneHelper(other.getContactPlaneHelper(i));
   }

   @Override
   public void setCommandId(int id)
   {
      commandId = id;
   }

   @Override
   public int getCommandId()
   {
      return commandId;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof ForceRateTrackingCommand)
      {
         ForceRateTrackingCommand other = (ForceRateTrackingCommand) object;
         if (commandId != other.commandId)
            return false;
         if (segmentNumber != other.segmentNumber)
            return false;
         if (omega != other.omega)
            return false;
         if (weight != other.weight)
            return false;
         if (duration != other.duration)
            return false;
         if (objectiveValue != other.objectiveValue)
            return false;
         if (contactPlanes.size() != other.contactPlanes.size())
            return false;
         for (int i = 0; i < contactPlanes.size(); i++)
         {
            if (!contactPlanes.get(i).equals(other.contactPlanes.get(i)))
               return false;
         }
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      String string = getClass().getSimpleName() + ": segment number: " + segmentNumber + ", omega: " + omega + ", weight: " + weight +
                      ": duration: " + duration + ": objective value: " + objectiveValue + ".";
      for (int i = 0; i < getNumberOfContacts(); i++)
      {
         string += "\ncontact " + i + ": " + contactPlanes.get(i);
      }
      return string;
   }
}
