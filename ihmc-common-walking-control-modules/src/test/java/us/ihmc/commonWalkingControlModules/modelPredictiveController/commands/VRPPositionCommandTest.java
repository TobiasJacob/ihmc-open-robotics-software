package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.yoVariables.registry.YoRegistry;

import static org.junit.jupiter.api.Assertions.*;

public class VRPPositionCommandTest
{
   @Test
   public void testCommandOptimize()
   {
      FramePoint3D objectivePosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.35, 0.7, 0.8);

      double gravityZ = -9.81;
      double omega = 3.0;
      double omega2 = omega * omega;
      double mu = 0.8;
      double dt = 1e-3;

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      CoefficientJacobianMatrixHelper helper = new CoefficientJacobianMatrixHelper(4, 4);
      MPCContactPlane contactPlaneHelper = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);
      LinearMPCQPSolver solver = new LinearMPCQPSolver(indexHandler, dt, gravityZ, new YoRegistry("test"));

      FramePose3D contactPose = new FramePose3D();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose, mu);
      rhoHelper.computeMatrices(contactPolygon, contactPose, 1e-8, 1e-10, mu);

      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 1);

      double timeOfConstraint = 0.7;

      VRPPositionCommand command = new VRPPositionCommand();
      command.setObjective(objectivePosition);
      command.setTimeOfObjective(timeOfConstraint);
      command.setSegmentNumber(0);
      command.setOmega(omega);
      command.setWeight(1.0);
      command.addContactPlaneHelper(contactPlaneHelper);

      double regularization = 1e-5;
      solver.initialize();
      solver.submitMPCValueObjective(command);
      solver.setComCoefficientRegularizationWeight(regularization);
      solver.setRhoCoefficientRegularizationWeight(regularization);

      solver.solve();

      FramePoint3D solvedPositionAtConstraint = new FramePoint3D();
      FramePoint3D solvedObjectivePositionTuple = new FramePoint3D();
      DMatrixRMaj rhoValueVector = new DMatrixRMaj(rhoHelper.getRhoSize(), 1);
      NativeMatrix solvedObjectivePosition = new NativeMatrix(3, 1);

      NativeMatrix solution = solver.getSolution();
      DMatrixRMaj rhoSolution = new DMatrixRMaj(rhoHelper.getRhoSize() * 4, 1);
      solvedPositionAtConstraint.addX((timeOfConstraint) * solution.get(0, 0));
      solvedPositionAtConstraint.addX(solution.get(1, 0));
      solvedPositionAtConstraint.addY((timeOfConstraint) * solution.get(2, 0));
      solvedPositionAtConstraint.addY(solution.get(3, 0));
      solvedPositionAtConstraint.addZ((timeOfConstraint) * solution.get(4, 0));
      solvedPositionAtConstraint.addZ(solution.get(5, 0));

      MatrixTools.setMatrixBlock(rhoSolution, 0, 0, solution, 6, 0, rhoHelper.getRhoSize() * 4, 1, 1.0);

      helper.computeMatrices(timeOfConstraint, omega);
      CommonOps_DDRM.mult(helper.getPositionJacobianMatrix(), rhoSolution, rhoValueVector);
      CommonOps_DDRM.multAdd(-1.0 / omega2, helper.getAccelerationJacobianMatrix(), rhoSolution, rhoValueVector);

      solvedObjectivePosition.mult(solver.qpInputTypeA.taskJacobian, solution);
      solvedObjectivePositionTuple.set(solvedObjectivePosition);
      solvedObjectivePositionTuple.scaleAdd(0.5 * timeOfConstraint * timeOfConstraint - 1.0 / omega2, gravityVector, solvedObjectivePositionTuple);

      DMatrixRMaj taskObjectiveExpected = new DMatrixRMaj(3, 1);
      NativeMatrix achievedObjective = new NativeMatrix(3, 1);
      objectivePosition.get(taskObjectiveExpected);
      taskObjectiveExpected.add(2, 0, (-0.5 * timeOfConstraint * timeOfConstraint + 1.0 / omega2) * -Math.abs(gravityZ));

      NativeMatrix taskJacobianExpected = MPCTestHelper.getVRPPositionJacobian(timeOfConstraint, omega, rhoHelper);

      for (int rhoIdx  = 0; rhoIdx < rhoHelper.getRhoSize(); rhoIdx++)
      {
         int startIdx = 6 + 4 * rhoIdx;
//         double rhoValue = Math.exp(omega * timeOfConstraint) * solution.get(startIdx, 0);
//         rhoValue += Math.exp(-omega * timeOfConstraint) * solution.get(startIdx + 1, 0);
         double rhoValue = (timeOfConstraint * timeOfConstraint * timeOfConstraint - 6.0 / omega2 * timeOfConstraint) * solution.get(startIdx + 2, 0);
         rhoValue += (timeOfConstraint * timeOfConstraint - 2.0 / omega2) * solution.get(startIdx + 3, 0);

         assertEquals(rhoValue, rhoValueVector.get(rhoIdx), 1e-5);
         solvedPositionAtConstraint.scaleAdd(rhoValue, rhoHelper.getBasisVector(rhoIdx), solvedPositionAtConstraint);
      }
      solvedPositionAtConstraint.scaleAdd(0.5 * timeOfConstraint * timeOfConstraint - 1.0 / omega2, gravityVector, solvedPositionAtConstraint);

      EjmlUnitTests.assertEquals(MPCTestHelper.getVRPPositionJacobian(timeOfConstraint, omega, contactPlaneHelper), solver.qpInputTypeA.taskJacobian, 1e-5);
      EjmlUnitTests.assertEquals(taskObjectiveExpected, solver.qpInputTypeA.taskObjective, 1e-5);

      achievedObjective.mult(new NativeMatrix(taskJacobianExpected), solution);
      EjmlUnitTests.assertEquals(taskObjectiveExpected, achievedObjective, 1e-4);

      NativeMatrix solverInput_H_Expected = new NativeMatrix(taskJacobianExpected.getNumCols(), taskJacobianExpected.getNumCols());
      NativeMatrix solverInput_f_Expected = new NativeMatrix(taskJacobianExpected.getNumCols(), 1);

      solverInput_H_Expected.multTransA(taskJacobianExpected, taskJacobianExpected);
      solverInput_f_Expected.multTransA(-1.0, taskJacobianExpected, new NativeMatrix(taskObjectiveExpected));

      MatrixTools.addDiagonal(solverInput_H_Expected, regularization);

      EjmlUnitTests.assertEquals(solverInput_H_Expected, solver.qpSolver.costQuadraticMatrix, 1e-10);
      EjmlUnitTests.assertEquals(solverInput_f_Expected, solver.qpSolver.quadraticCostQVector, 1e-10);


      EuclidCoreTestTools.assertTuple3DEquals(objectivePosition, solvedObjectivePositionTuple, 1e-4);
      EuclidCoreTestTools.assertTuple3DEquals(objectivePosition, solvedPositionAtConstraint, 1e-4);
   }

}
