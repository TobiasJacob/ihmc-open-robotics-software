package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;

import java.util.List;

public class VRPTrajectorySubspaceSolver
{
   private final RecyclingArrayList<DMatrixRMaj> xCoefficients = new RecyclingArrayList<>(() -> new DMatrixRMaj(4, 1));
   private final RecyclingArrayList<DMatrixRMaj> yCoefficients = new RecyclingArrayList<>(() -> new DMatrixRMaj(4, 1));
   private final RecyclingArrayList<DMatrixRMaj> zCoefficients = new RecyclingArrayList<>(() -> new DMatrixRMaj(4, 1));

   private final DMatrixRMaj vrpConstraintMatrix = new DMatrixRMaj(4, 4);
   private final DMatrixRMaj vrpConstraintMatrixInverse = new DMatrixRMaj(4, 4);
   private final DMatrixRMaj vrpBoundsVector = new DMatrixRMaj(4, 1);

   private final LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.general(4, 4);

   public void solveForCoefficientSubspaceCoefficients(List<FramePoint3DReadOnly> startVRPPositions,
                                                       List<FramePoint3DReadOnly> endVRPPositions,
                                                       List<? extends ContactStateProvider> contactSequence,
                                                       double omega)
   {
      xCoefficients.clear();
      yCoefficients.clear();
      zCoefficients.clear();

      for (int i = 0; i < contactSequence.size(); i++)
         solveForCoefficientSubspaceCoefficients(startVRPPositions.get(i), endVRPPositions.get(i), contactSequence.get(i), omega);
   }

   private void solveForCoefficientSubspaceCoefficients(FramePoint3DReadOnly startVRPPosition,
                                                        FramePoint3DReadOnly endVRPPosition,
                                                        ContactStateProvider contact,
                                                        double omega)
   {
      computeVRPConstraintMatrix(omega, contact.getTimeInterval().getDuration());

      vrpBoundsVector.set(0, 0, startVRPPosition.getX());
      vrpBoundsVector.set(0, 0, contact.getECMPStartVelocity().getX());
      vrpBoundsVector.set(0, 0, endVRPPosition.getX());
      vrpBoundsVector.set(0, 0, contact.getECMPEndVelocity().getX());

      CommonOps_DDRM.mult(vrpConstraintMatrixInverse, vrpBoundsVector, xCoefficients.add());

      vrpBoundsVector.set(0, 0, startVRPPosition.getY());
      vrpBoundsVector.set(0, 0, contact.getECMPStartVelocity().getY());
      vrpBoundsVector.set(0, 0, endVRPPosition.getY());
      vrpBoundsVector.set(0, 0, contact.getECMPEndVelocity().getY());

      CommonOps_DDRM.mult(vrpConstraintMatrixInverse, vrpBoundsVector, yCoefficients.add());

      vrpBoundsVector.set(0, 0, startVRPPosition.getZ());
      vrpBoundsVector.set(0, 0, contact.getECMPStartVelocity().getZ());
      vrpBoundsVector.set(0, 0, endVRPPosition.getZ());
      vrpBoundsVector.set(0, 0, contact.getECMPEndVelocity().getZ());

      CommonOps_DDRM.mult(vrpConstraintMatrixInverse, vrpBoundsVector, zCoefficients.add());
   }

   private void computeVRPConstraintMatrix(double omega, double duration)
   {
      vrpConstraintMatrix.zero();

      double omega2 = omega * omega;
      double t2 = duration * duration;
      double t3 = t2 * duration;

      // start VRP position
      vrpConstraintMatrix.set(0, 1, -2.0 / omega2);
      vrpConstraintMatrix.set(0, 3, 1.0);

      // start VRP velocity
      vrpConstraintMatrix.set(1, 0, -6.0 / omega2);
      vrpConstraintMatrix.set(1, 2, 1.0);

      // end VRP position
      vrpConstraintMatrix.set(2, 0, t3 - 6.0 * duration / omega2);
      vrpConstraintMatrix.set(2, 1, t2 - 2.0 / omega2);
      vrpConstraintMatrix.set(2, 2, duration);
      vrpConstraintMatrix.set(2, 3, 1.0);

      // end VRP velocity
      vrpConstraintMatrix.set(3, 0, 3.0 * t2 - 6.0 / omega2);
      vrpConstraintMatrix.set(3, 1, 2.0 * duration);
      vrpConstraintMatrix.set(3, 2, 1.0);

      solver.setA(vrpConstraintMatrix);
      solver.invert(vrpConstraintMatrixInverse);
   }
}
