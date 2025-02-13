package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrix;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPoint;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.VRPTrackingCommand;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.MatrixMissingTools;

import java.util.ArrayList;
import java.util.List;

/**
 * This class is used to compute the cost functions for tracking a desired VRP function.
 * This tracking function can be computed as a convex quadratic cost term.
 *
 * TODO review class to make it more efficient for column-wise operations, such as what happens in the sparse matrices
 */
public class VRPTrackingCostCalculator
{
   private final LinearMPCIndexHandler indexHandler;
   private final double gravityZ;

   private final List<FrameVector3DReadOnly> allBasisVectors = new ArrayList<>();
   private final FrameVector3D vrpChange = new FrameVector3D();

   private final FrameVector3D c0Desired = new FrameVector3D();
   private final FrameVector3D c1Desired = new FrameVector3D();
   private final FrameVector3D c2Desired = new FrameVector3D();
   private final FrameVector3D c3Desired = new FrameVector3D();

   private final FrameVector3D a0Desired = new FrameVector3D();
   private final FrameVector3D a1Desired = new FrameVector3D();
   private final FrameVector3D a2Desired = new FrameVector3D();
   private final FrameVector3D a3Desired = new FrameVector3D();

   private final FramePoint3D desiredValuePosition = new FramePoint3D();
   private final FramePoint3D desiredValueVelocity = new FramePoint3D();

   public VRPTrackingCostCalculator(LinearMPCIndexHandler indexHandler, double gravityZ)
   {
      this.indexHandler = indexHandler;
      this.gravityZ = -Math.abs(gravityZ);
   }

   /**
    * Calculates the quadratic cost function for tracking a nominal VRP trajectory, which is specified by the {@link VRPTrackingCommand}
    *
    * @param costHessianToPack hessian of the quadratic cost function to pack
    * @param costGradientToPack gradient of the quadratic cost function to pack
    * @param objective objective object containing the desired VRP trajectory information
    * @return whether the computation was successful
    */
   public boolean calculateVRPTrackingObjective(DMatrix costHessianToPack, DMatrix costGradientToPack, VRPTrackingCommand objective)
   {
      int segmentNumber = objective.getSegmentNumber();
      int startCoMIdx = indexHandler.getComCoefficientStartIndex(segmentNumber, 0);
      int startRhoIdx = indexHandler.getRhoCoefficientStartIndex(segmentNumber);

      if (!hasValidVelocityBounds(objective))
      {
         velocity.sub(objective.getEndVRP(), objective.getStartVRP());
         velocity.scale(1.0 / (objective.getEndTime() - objective.getStartTime()));
         objective.setStartVRPVelocity(velocity);
         objective.setEndVRPVelocity(velocity);
      }

      return calculateCubicVRPTrackingObjectiveInternal(costHessianToPack, costGradientToPack, objective, startCoMIdx, startRhoIdx);
   }

   private final FrameVector3D velocity = new FrameVector3D();

   public boolean calculateCompactVRPTrackingObjective(DMatrix costHessianToPack, DMatrix costGradientToPack, VRPTrackingCommand objective)
   {
      if (!hasValidVelocityBounds(objective))
      {
         velocity.sub(objective.getEndVRP(), objective.getStartVRP());
         velocity.scale(1.0 / (objective.getEndTime() - objective.getStartTime()));
         objective.setStartVRPVelocity(velocity);
         objective.setEndVRPVelocity(velocity);
      }

      return calculateCubicVRPTrackingObjectiveInternal(costHessianToPack, costGradientToPack, objective, 0, LinearMPCIndexHandler.comCoefficientsPerSegment);
   }

   private static boolean hasValidVelocityBounds(VRPTrackingCommand objective)
   {
      return !objective.getStartVRPVelocity().containsNaN() && !objective.getEndVRPVelocity().containsNaN();
   }

   private boolean calculateLinearVRPTrackingObjectiveInternal(DMatrix costHessianToPack,
                                                               DMatrix costGradientToPack,
                                                               VRPTrackingCommand objective,
                                                               int startCoMIdx,
                                                               int startRhoIdx)
   {
      double omega = objective.getOmega();
      double w2 = omega * omega;
      double w4 = w2 * w2;

      double tEnd = objective.getEndTime();
      double tStart = objective.getStartTime();

      double duration = tEnd - tStart;

      double t2End = tEnd * tEnd;
      double t3End = tEnd * t2End;
      double t4End = tEnd * t3End;
      double t5End = tEnd * t4End;
      double t6End = tEnd * t5End;
      double t7End = tEnd * t6End;

      double t2Start = tStart * tStart;
      double t3Start = tStart * t2Start;
      double t4Start = tStart * t3Start;
      double t5Start = tStart * t4Start;
      double t6Start = tStart * t5Start;
      double t7Start = tStart * t6Start;

      double c0c0 = (t3End - t3Start) / 3.0;
      double c0c1 = (t2End - t2Start) / 2.0;
      double c1c1 = duration;

      double gc0 = (t4End - t4Start) / 8.0 - 0.5 * (t2End - t2Start) / w2;
      double gc1 = (t3End - t3Start) / 6.0 - (tEnd - tStart) / w2;


      costHessianToPack.set(startCoMIdx, startCoMIdx, c0c0);
      costHessianToPack.set(startCoMIdx, startCoMIdx + 1, c0c1);
      costHessianToPack.set(startCoMIdx + 1, startCoMIdx, c0c1);
      costHessianToPack.set(startCoMIdx + 1, startCoMIdx + 1, c1c1);

      costHessianToPack.set(startCoMIdx + 2, startCoMIdx + 2, c0c0);
      costHessianToPack.set(startCoMIdx + 2, startCoMIdx + 3, c0c1);
      costHessianToPack.set(startCoMIdx + 3, startCoMIdx + 2, c0c1);
      costHessianToPack.set(startCoMIdx + 3, startCoMIdx + 3, duration);

      costHessianToPack.set(startCoMIdx + 4, startCoMIdx + 4, c0c0);
      costHessianToPack.set(startCoMIdx + 4, startCoMIdx + 5, c0c1);
      costHessianToPack.set(startCoMIdx + 5, startCoMIdx + 4, c0c1);
      costHessianToPack.set(startCoMIdx + 5, startCoMIdx + 5, duration);

      costGradientToPack.set(startCoMIdx + 4, 0, gc0 * gravityZ);
      costGradientToPack.set(startCoMIdx + 5, 0, gc1 * gravityZ);

      allBasisVectors.clear();
      for (int contactPlaneIdx = 0; contactPlaneIdx < objective.getNumberOfContacts(); contactPlaneIdx++)
      {
         MPCContactPlane contactPlane = objective.getContactPlaneHelper(contactPlaneIdx);
         for (int contactPointIdx = 0; contactPointIdx < contactPlane.getNumberOfContactPoints(); contactPointIdx++)
         {
            MPCContactPoint contactPoint = contactPlane.getContactPointHelper(contactPointIdx);
            for (int i = 0; i < contactPoint.getRhoSize(); i++)
            {
               allBasisVectors.add(contactPoint.getBasisVector(i));
            }
         }
      }

      double a2a2 = (t7End - t7Start) / 7.0 - (t5End - t5Start) * 12.0 / (5.0 * w2) + 12.0 * (t3End - t3Start) / w4;
      double a2a3 = (t6End - t6Start) / 6.0 - 2.0 * (t4End - t4Start) / w2 + 6.0 / w4 * (t2End - t2Start);
      double a3a3 = (t5End - t5Start) / 5.0 - 4.0 / 3.0 * (t3End - t3Start) / w2 + 4.0 / w4 * (tEnd - tStart);

      double a2c0 = (t5End - t5Start) / 5.0 - 2.0 * (t3End - t3Start) / w2;
      double a3c0 = (t4End - t4Start) / 4.0 - (t2End - t2Start) / w2;
      double a2c1 = (t4End - t4Start) / 4.0 - 3.0 * (t2End - t2Start) / w2;
      double a3c1 = (t3End - t3Start) / 3.0 - 2.0 * (tEnd - tStart) / w2;


      double a2c2Desired = (t5End - t5Start) / 5.0 - 2.0 * (t3End - t3Start) / w2;
      double a3c2Desired = (t4End - t4Start) / 4.0 - (t2End - t2Start) / w2;
      double a2c3Desired = (t4End - t4Start) / 4.0 - 3.0 * (t2End - t2Start) / w2;
      double a3c3Desired = (t3End - t3Start) / 3.0 - 2.0 * (tEnd - tStart) / w2;

      double ga2 = (t6End - t6Start) / 12.0 - (t4End - t4Start) / w2 + 3.0 * (t2End - t2Start) / w4;
      double ga3 = (t5End - t5Start) / 10.0 - 2.0 * (t3End - t3Start) / (3.0 * w2) + 2.0 / w4 * (tEnd - tStart);

      c2Desired.sub(objective.getEndVRP(), objective.getStartVRP());
      c2Desired.scale(duration);
      c3Desired.set(c2Desired);
      c3Desired.scaleAdd(-tStart, objective.getStartVRP());

      // TODO review to see if the set vs add methods are correct
      for (int ordinal = 0; ordinal < 3; ordinal++)
      {
         int offset = 2 * ordinal + startCoMIdx;
         double c0 = (t3End - t3Start) / 3.0 * c2Desired.getElement(ordinal) + (t2End - t2Start) / 2.0 * c3Desired.getElement(ordinal);
         double c1 = (t2End - t2Start) / 2.0 * c2Desired.getElement(ordinal) + (tEnd - tStart) * c3Desired.getElement(ordinal);

         MatrixMissingTools.unsafe_add(costGradientToPack, offset, 0, -c0);
         MatrixMissingTools.unsafe_add(costGradientToPack, offset + 1, 0, -c1);
      }

      for (int i = 0; i < allBasisVectors.size(); i++)
      {
         int idxI = 4 * i + startRhoIdx + 2;

         FrameVector3DReadOnly basisVector = allBasisVectors.get(i);

         MatrixMissingTools.unsafe_add(costHessianToPack, idxI, idxI, a2a2);
         MatrixMissingTools.unsafe_add(costHessianToPack, idxI, idxI + 1, a2a3);
         MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, idxI, a2a3);
         MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, idxI + 1, a3a3);

         for (int j = i + 1; j < allBasisVectors.size(); j++)
         {
            FrameVector3DReadOnly otherBasisVector = allBasisVectors.get(j);

            double basisDot = basisVector.dot(otherBasisVector);

            int idxJ = 4 * j + startRhoIdx + 2;

            MatrixMissingTools.unsafe_add(costHessianToPack, idxI, idxJ, basisDot * a2a2);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI, idxJ + 1, basisDot * a2a3);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, idxJ, basisDot * a2a3);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, idxJ + 1, basisDot * a3a3);

            // we know it's symmetric, and this way we can avoid iterating as much
            MatrixMissingTools.unsafe_add(costHessianToPack, idxJ, idxI, basisDot * a2a2);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxJ + 1, idxI, basisDot * a2a3);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxJ, idxI + 1, basisDot * a2a3);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxJ + 1, idxI + 1, basisDot * a3a3);
         }


         for (int ordinal = 0; ordinal < 3; ordinal++)
         {
            int offset = startCoMIdx + 2 * ordinal;
            double value = basisVector.getElement(ordinal);
            MatrixMissingTools.unsafe_add(costHessianToPack, offset, idxI, a2c0 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, offset, idxI + 1, a3c0 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, offset + 1, idxI, a2c1 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, offset + 1, idxI + 1, a3c1 * value);

            // symmetric...
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI, offset, a2c0 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, offset,  a3c0 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI, offset + 1, a2c1 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, offset + 1, a3c1 * value);
         }

         double basisDotC2 = c2Desired.dot(basisVector);
         double basisDotC3 = c3Desired.dot(basisVector);
         double basisDotG = basisVector.getZ() * gravityZ;

         MatrixMissingTools.unsafe_add(costGradientToPack, idxI, 0, -basisDotC2 * a2c2Desired - basisDotC3 * a2c3Desired + basisDotG * ga2);
         MatrixMissingTools.unsafe_add(costGradientToPack, idxI + 1, 0, -basisDotC2 * a3c2Desired - basisDotC3 * a3c3Desired + basisDotG * ga3);
      }

      return true;
   }

   private boolean calculateCubicVRPTrackingObjectiveInternal(DMatrix costHessianToPack,
                                                               DMatrix costGradientToPack,
                                                               VRPTrackingCommand objective,
                                                               int startCoMIdx,
                                                               int startRhoIdx)
   {
      double omega = objective.getOmega();
      double w2 = omega * omega;
      double w4 = w2 * w2;


      double tEnd = objective.getEndTime();
      double t2End = tEnd * tEnd;
      double t3End = tEnd * t2End;
      double t4End = tEnd * t3End;
      double t5End = tEnd * t4End;
      double t6End = tEnd * t5End;
      double t7End = tEnd * t6End;

      double tStart = objective.getStartTime();
      double t2Start = tStart * tStart;
      double t3Start = tStart * t2Start;
      double t4Start = tStart * t3Start;
      double t5Start = tStart * t4Start;
      double t6Start = tStart * t5Start;
      double t7Start = tStart * t6Start;

      double t7Change = t7End - t7Start;
      double t6Change = t6End - t6Start;
      double t5Change = t5End - t5Start;
      double t4Change = t4End - t4Start;
      double t3Change = t3End - t3Start;
      double t2Change = t2End - t2Start;
      double tChange = tEnd - tStart;

      double c0c0 = t3Change / 3.0;
      double c0c1 = 0.5 * t2Change;

      double gc0 = t4Change / 8.0 - 0.5 * t2Change / w2;
      double gc1 = t3Change / 6.0 - tChange / w2;

      costHessianToPack.set(startCoMIdx, startCoMIdx, c0c0);
      costHessianToPack.set(startCoMIdx, startCoMIdx + 1, c0c1);
      costHessianToPack.set(startCoMIdx + 1, startCoMIdx, c0c1);
      costHessianToPack.set(startCoMIdx + 1, startCoMIdx + 1, tChange);

      costHessianToPack.set(startCoMIdx + 2, startCoMIdx + 2, c0c0);
      costHessianToPack.set(startCoMIdx + 2, startCoMIdx + 3, c0c1);
      costHessianToPack.set(startCoMIdx + 3, startCoMIdx + 2, c0c1);
      costHessianToPack.set(startCoMIdx + 3, startCoMIdx + 3, tChange);

      costHessianToPack.set(startCoMIdx + 4, startCoMIdx + 4, c0c0);
      costHessianToPack.set(startCoMIdx + 4, startCoMIdx + 5, c0c1);
      costHessianToPack.set(startCoMIdx + 5, startCoMIdx + 4, c0c1);
      costHessianToPack.set(startCoMIdx + 5, startCoMIdx + 5, tChange);

      costGradientToPack.set(startCoMIdx + 4, 0, gc0 * gravityZ);
      costGradientToPack.set(startCoMIdx + 5, 0, gc1 * gravityZ);

      allBasisVectors.clear();
      for (int contactPlaneIdx = 0; contactPlaneIdx < objective.getNumberOfContacts(); contactPlaneIdx++)
      {
         MPCContactPlane contactPlane = objective.getContactPlaneHelper(contactPlaneIdx);
         for (int contactPointIdx = 0; contactPointIdx < contactPlane.getNumberOfContactPoints(); contactPointIdx++)
         {
            MPCContactPoint contactPoint = contactPlane.getContactPointHelper(contactPointIdx);
            for (int i = 0; i < contactPoint.getRhoSize(); i++)
            {
               allBasisVectors.add(contactPoint.getBasisVector(i));
            }
         }
      }

      double a2c0Desired = t7Change / 7.0 - 6.0 * t5Change / (5.0 * w2);
      double a3c0Desired = t6Change / 6.0 - t4Change / (2.0 * w2);
      double a2c1Desired = t6Change / 6.0 - 3.0 * t4Change / (2.0 * w2);
      double a3c1Desired = t5Change / 5.0 - 2.0 * t3Change / (3.0 * w2);
      double a2c2Desired = t5Change / 5.0 - 2.0 * t3Change / w2;
      double a3c2Desired = t4Change / 4.0 - t2Change / w2;
      double a2c3Desired = t4Change / 4.0 - 3.0 * t2Change / w2;
      double a3c3Desired = t3Change / 3.0 - 2.0 * tChange / w2;

      double ga2 = t6Change / 12.0 - t4Change / w2 + 3.0 * t2Change / w4;
      double ga3 = t5Change / 10.0 - 2.0 * t3Change / (3.0 * w2) + 2.0 / w4 * tChange;

      double a2a2 = t7Change / 7.0 - 12.0 * t5Change / (5.0 * w2) + 12.0 / w4 * t3Change;
      double a2a3 = t6Change / 6.0 - 2.0 * t4Change / w2 + 6.0 / w4 * t2Change;
      double a3a3 = t5Change / 5.0 - 4.0 / 3.0 * t3Change / w2 + 4.0 / w4 * tChange;

      double a2c0 = t5Change / 5.0 - 2.0 * t3Change / w2;
      double a3c0 = t4Change / 4.0 - t2Change / w2;
      double a2c1 = t4Change / 4.0 - 3.0 * t2Change / w2;
      double a3c1 = t3Change / 3.0 - 2.0 * tChange / w2;

      // compute the cubic function in terms of the time relative to the start of the segment
      vrpChange.sub(objective.getEndVRP(), objective.getStartVRP());

      double d2 = tChange * tChange;
      double d3 = tChange * d2;
      a0Desired.add(objective.getEndVRPVelocity(), objective.getStartVRPVelocity());
      a0Desired.scale(1.0 / d2);
      a0Desired.scaleAdd(-2.0 / d3, vrpChange, a0Desired);

      a1Desired.add(objective.getEndVRPVelocity(), objective.getStartVRPVelocity());
      a1Desired.add(objective.getStartVRPVelocity());
      a1Desired.scale(-1.0 / tChange);
      a1Desired.scaleAdd(3.0 / d2, vrpChange, a1Desired);

      a2Desired.set(objective.getStartVRPVelocity());

      a3Desired.set(objective.getStartVRP());

      // convert to global time.
      c0Desired.set(a0Desired);

      c1Desired.setAndScale(-3.0 * tStart, a0Desired);
      c1Desired.add(a1Desired);

      c2Desired.setAndScale(3.0 * t2Start, a0Desired);
      c2Desired.scaleAdd(-2.0 * tStart, a1Desired, c2Desired);
      c2Desired.add(a2Desired);

      c3Desired.setAndScale(-t3Start, a0Desired);
      c3Desired.scaleAdd(t2Start, a1Desired, c3Desired);
      c3Desired.scaleAdd(-tStart, a2Desired, c3Desired);
      c3Desired.add(a3Desired);

      desiredValuePosition.setAndScale(t5Change / 5.0, c0Desired);
      desiredValuePosition.scaleAdd(t4Change / 4.0, c1Desired, desiredValuePosition);
      desiredValuePosition.scaleAdd(t3Change / 3.0, c2Desired, desiredValuePosition);
      desiredValuePosition.scaleAdd(t2Change / 2.0, c3Desired, desiredValuePosition);

      desiredValueVelocity.setAndScale(t4Change / 4.0, c0Desired);
      desiredValueVelocity.scaleAdd(t3Change / 3.0, c1Desired, desiredValueVelocity);
      desiredValueVelocity.scaleAdd(t2Change / 2.0, c2Desired, desiredValueVelocity);
      desiredValueVelocity.scaleAdd(tChange, c3Desired, desiredValueVelocity);

      // TODO review to see if the set vs add methods are correct
      for (int ordinal = 0; ordinal < 3; ordinal++)
      {
         int offset = 2 * ordinal + startCoMIdx;

         MatrixMissingTools.unsafe_add(costGradientToPack, offset, 0, -desiredValuePosition.getElement(ordinal));
         MatrixMissingTools.unsafe_add(costGradientToPack, offset + 1, 0, -desiredValueVelocity.getElement(ordinal));
      }

      for (int i = 0; i < allBasisVectors.size(); i++)
      {
         int idxI = 4 * i + startRhoIdx + 2;

         FrameVector3DReadOnly basisVector = allBasisVectors.get(i);

         MatrixMissingTools.unsafe_add(costHessianToPack, idxI, idxI, a2a2);
         MatrixMissingTools.unsafe_add(costHessianToPack, idxI, idxI + 1, a2a3);
         MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, idxI, a2a3);
         MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, idxI + 1, a3a3);

         for (int j = i + 1; j < allBasisVectors.size(); j++)
         {
            FrameVector3DReadOnly otherBasisVector = allBasisVectors.get(j);

            double basisDot = basisVector.dot(otherBasisVector);

            int idxJ = 4 * j + startRhoIdx + 2;

            MatrixMissingTools.unsafe_add(costHessianToPack, idxI, idxJ, basisDot * a2a2);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI, idxJ + 1, basisDot * a2a3);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, idxJ, basisDot * a2a3);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, idxJ + 1, basisDot * a3a3);

            // we know it's symmetric, and this way we can avoid iterating as much
            MatrixMissingTools.unsafe_add(costHessianToPack, idxJ, idxI, basisDot * a2a2);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxJ + 1, idxI, basisDot * a2a3);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxJ, idxI + 1, basisDot * a2a3);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxJ + 1, idxI + 1, basisDot * a3a3);
         }


         for (int ordinal = 0; ordinal < 3; ordinal++)
         {
            int offset = startCoMIdx + 2 * ordinal;
            double value = basisVector.getElement(ordinal);
            MatrixMissingTools.unsafe_add(costHessianToPack, offset, idxI, a2c0 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, offset, idxI + 1, a3c0 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, offset + 1, idxI, a2c1 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, offset + 1, idxI + 1, a3c1 * value);

            // symmetric...
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI, offset, a2c0 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, offset,  a3c0 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI, offset + 1, a2c1 * value);
            MatrixMissingTools.unsafe_add(costHessianToPack, idxI + 1, offset + 1, a3c1 * value);
         }

         double basisDotC0 = c0Desired.dot(basisVector);
         double basisDotC1 = c1Desired.dot(basisVector);
         double basisDotC2 = c2Desired.dot(basisVector);
         double basisDotC3 = c3Desired.dot(basisVector);
         double basisDotG = basisVector.getZ() * gravityZ;

         MatrixMissingTools.unsafe_add(costGradientToPack, idxI, 0, basisDotG * ga2 - basisDotC0 * a2c0Desired - basisDotC1 * a2c1Desired - basisDotC2 * a2c2Desired - basisDotC3 * a2c3Desired);
         MatrixMissingTools.unsafe_add(costGradientToPack, idxI + 1, 0, basisDotG * ga3 - basisDotC0 * a3c0Desired - basisDotC1 * a3c1Desired
                                                                        -basisDotC2 * a3c2Desired - basisDotC3 * a3c3Desired);
      }

      return true;
   }
}
