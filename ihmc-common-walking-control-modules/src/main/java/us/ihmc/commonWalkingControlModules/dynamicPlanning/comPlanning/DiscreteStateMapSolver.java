package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;

import java.util.List;

public class DiscreteStateMapSolver
{
   private final VRPTrajectorySubspaceSolver vrpTrajectorySubspaceSolver;

   private final RecyclingArrayList<DMatrixRMaj> stateMapsToNextSegment = new RecyclingArrayList<>(() -> new DMatrixRMaj(2, 2));

   private final RecyclingArrayList<DMatrixRMaj> stateConstantToNextSegmentForCurrent = new RecyclingArrayList<>(() -> new DMatrixRMaj(2, 4));
   private final DMatrixRMaj stateConstantToNextSegmentForNext = new DMatrixRMaj(2, 4);

   private final RecyclingArrayList<DMatrixRMaj> stateMapsToSegmentFromCurrent = new RecyclingArrayList<>(() -> new DMatrixRMaj(2, 2));
   private final RecyclingArrayList<DMatrixRMaj> stateXConstantsFromCurrent = new RecyclingArrayList<>(() -> new DMatrixRMaj(2, 1));
   private final RecyclingArrayList<DMatrixRMaj> stateYConstantsFromCurrent = new RecyclingArrayList<>(() -> new DMatrixRMaj(2, 1));
   private final RecyclingArrayList<DMatrixRMaj> stateZConstantsFromCurrent = new RecyclingArrayList<>(() -> new DMatrixRMaj(2, 1));

   private final DMatrixRMaj tauFinal = new DMatrixRMaj(2, 4);
   private final DMatrixRMaj tauInitial = new DMatrixRMaj(2, 4);

   private final DMatrixRMaj stateInitialMapInverse = new DMatrixRMaj(2, 2);

   public DiscreteStateMapSolver(VRPTrajectorySubspaceSolver vrpTrajectorySubspaceSolver)
   {
      this.vrpTrajectorySubspaceSolver = vrpTrajectorySubspaceSolver;

      tauInitial.set(0, 3, 1.0);
      tauInitial.set(1, 2, 1.0);

      stateInitialMapInverse.set(0, 0, 0.5);
      stateInitialMapInverse.set(1, 0, 0.5);
   }

   public void solveForStateMaps(List<? extends ContactStateProvider> contactSequence, double omega)
   {
      stateInitialMapInverse.set(0, 1, 1.0 / (2.0 * omega));
      stateInitialMapInverse.set(1, 1, -1.0 / (2.0 * omega));

      solveForStateMapsAndConstantsToNextSegment(omega, contactSequence);
      CommonOps_DDRM.mult(-1.0, stateInitialMapInverse, tauInitial, stateConstantToNextSegmentForNext);

      solveForStateMapsAndConstantsFromCurrent();
   }

   private void solveForStateMapsAndConstantsToNextSegment(double omega, List<? extends ContactStateProvider> contactSequence)
   {
      for (int i = 0; i < contactSequence.size() - 1; i++)
      {
         double duration = contactSequence.get(i).getTimeInterval().getDuration();
         solveForStateMapToNextSegment(omega, duration);
         solveForStateConstantToNextSegmentForCurrent(duration);
      }
   }

   private void solveForStateMapToNextSegment(double omega, double duration)
   {
      DMatrixRMaj map = stateMapsToNextSegment.add();

      double term = Math.exp(omega * duration);
      map.zero();
      map.set(0, 0, term);
      map.set(1, 1, 1.0 / term);
   }

   private void solveForStateConstantToNextSegmentForCurrent(double duration)
   {
      double t2 = duration * duration;
      double t3 = duration * t2;

      tauFinal.set(0, 0, t3);
      tauFinal.set(0, 1, t2);
      tauFinal.set(0, 2, duration);
      tauFinal.set(0, 3, 1.0);

      tauFinal.set(1, 0, 3.0 * t2);
      tauFinal.set(1, 1, 2.0 * duration);
      tauFinal.set(1, 2, 1.0);

      CommonOps_DDRM.mult(stateInitialMapInverse, tauFinal, stateConstantToNextSegmentForCurrent.add());
   }
   

   private void solveForStateMapsAndConstantsFromCurrent(double omega, List<? extends ContactStateProvider> contactSequence)
   {
      stateMapsToSegmentFromCurrent.clear();

      // solve for state maps
      double totalDuration = 0.0;
      for (int i = 0; i < contactSequence.size() - 1; i++)
      {
         // FIXME DO SOMETHING DIFFERENT IN FLIGHT, WHICH BREAKS THIS
         totalDuration += contactSequence.get(i).getTimeInterval().getDuration();
         double cumulativeExponential = Math.exp(omega * totalDuration);

         DMatrixRMaj matrix = stateMapsToSegmentFromCurrent.add();
         matrix.set(0, 0, cumulativeExponential);
         matrix.set(1, 1, 1.0 / cumulativeExponential);

         DMatrixRMaj nextX = stateXConstantsFromCurrent.add();
         DMatrixRMaj nextY = stateYConstantsFromCurrent.add();
         DMatrixRMaj nextZ = stateZConstantsFromCurrent.add();

         CommonOps_DDRM.mult(stateConstantToNextSegmentForCurrent.get(i), vrpTrajectorySubspaceSolver.getXCoefficients(i), nextX);
         CommonOps_DDRM.multAdd(stateConstantToNextSegmentForNext, vrpTrajectorySubspaceSolver.getXCoefficients(i + 1), nextX);

         CommonOps_DDRM.mult(stateConstantToNextSegmentForCurrent.get(i), vrpTrajectorySubspaceSolver.getXCoefficients(i), nextY);
         CommonOps_DDRM.multAdd(stateConstantToNextSegmentForNext, vrpTrajectorySubspaceSolver.getXCoefficients(i + 1), nextY);

         CommonOps_DDRM.mult(stateConstantToNextSegmentForCurrent.get(i), vrpTrajectorySubspaceSolver.getXCoefficients(i), nextZ);
         CommonOps_DDRM.multAdd(stateConstantToNextSegmentForNext, vrpTrajectorySubspaceSolver.getXCoefficients(i + 1), nextZ);

         if (i > 0)
         {
            DMatrixRMaj previousX = stateXConstantsFromCurrent.get(i - 1);
            DMatrixRMaj previousY = stateYConstantsFromCurrent.get(i - 1);
            DMatrixRMaj previousZ = stateZConstantsFromCurrent.get(i - 1);

            double currentExponential = Math.exp(omega * contactSequence.get(i).getTimeInterval().getDuration());

            nextX.add(0, 0, previousX.get(0, 0) * currentExponential);
            nextX.add(1, 0, previousX.get(1, 0) / currentExponential);

            nextY.add(0, 0, previousY.get(0, 0) * currentExponential);
            nextY.add(1, 0, previousY.get(1, 0) / currentExponential);

            nextZ.add(0, 0, previousZ.get(0, 0) * currentExponential);
            nextZ.add(1, 0, previousZ.get(1, 0) / currentExponential);
         }
      }
   }
}
