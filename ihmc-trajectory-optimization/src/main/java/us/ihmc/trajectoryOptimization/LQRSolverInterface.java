package us.ihmc.trajectoryOptimization;

import org.ejml.data.DMatrixRMaj;

public interface LQRSolverInterface<E extends Enum>
{
   void setDesiredSequence(DiscreteOptimizationData desiredSequence, DiscreteSequence constantsSequence, DMatrixRMaj initialState);
   void getOptimalSequence(DiscreteOptimizationData optimalSequenceToPack);

   DiscreteOptimizationData getOptimalSequence();
   DiscreteData getOptimalStateSequence();
   DiscreteData getOptimalControlSequence();
   DiscreteSequence getOptimalFeedbackGainSequence();
   DiscreteSequence getOptimalFeedForwardControlSequence();

   DMatrixRMaj getValueHessian();

   void solveRiccatiEquation(E dynamicState, int startIndex, int endIndex); // backwards pass
   void computeOptimalSequences(E dynamicsState, int startIndex, int endIndex); // forward pass
}
