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

public class VRPVelocityCommandTest
{
   @Test
   public void testCommandOptimize()
   {
      FramePoint3D objectiveVelocity = new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.35, 0.7, 0.8);

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
      rhoHelper.computeMatrices(contactPolygon, contactPose, 1e-8, 1e-10, mu);
      contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose, mu);

      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 1);

      double timeOfConstraint = 0.7;

      VRPVelocityCommand command = new VRPVelocityCommand();
      command.setObjective(objectiveVelocity);
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

      FramePoint3D solvedVelocityAtConstraint = new FramePoint3D();
      FramePoint3D solvedObjectiveVelocityTuple = new FramePoint3D();
      DMatrixRMaj rhoValueVector = new DMatrixRMaj(rhoHelper.getRhoSize(), 1);
      NativeMatrix solvedObjectiveVelocity = new NativeMatrix(3, 1);

      NativeMatrix solution = solver.getSolution();
      DMatrixRMaj rhoSolution = new DMatrixRMaj(rhoHelper.getRhoSize() * 4, 1);
      solvedVelocityAtConstraint.addX(solution.get(0, 0));
      solvedVelocityAtConstraint.addY(solution.get(2, 0));
      solvedVelocityAtConstraint.addZ(solution.get(4, 0));

      MatrixTools.setMatrixBlock(rhoSolution, 0, 0, solution, 6, 0, rhoHelper.getRhoSize() * 4, 1, 1.0);

      helper.computeMatrices(timeOfConstraint, omega);
      CommonOps_DDRM.mult(helper.getVelocityJacobianMatrix(), rhoSolution, rhoValueVector);
      CommonOps_DDRM.multAdd(-1.0 / omega2, helper.getJerkJacobianMatrix(), rhoSolution, rhoValueVector);

      solvedObjectiveVelocity.mult(solver.qpInputTypeA.taskJacobian, solution);
      solvedObjectiveVelocityTuple.set(solvedObjectiveVelocity);
      solvedObjectiveVelocityTuple.scaleAdd(timeOfConstraint, gravityVector, solvedObjectiveVelocityTuple);

      NativeMatrix taskObjectiveExpected = new NativeMatrix(3, 1);
      NativeMatrix achievedObjective = new NativeMatrix(3, 1);
      objectiveVelocity.get(taskObjectiveExpected);
      taskObjectiveExpected.add(2, 0, -timeOfConstraint * -Math.abs(gravityZ));

      NativeMatrix taskJacobianExpected = MPCTestHelper.getVRPVelocityJacobian(timeOfConstraint, omega, rhoHelper);


      for (int rhoIdx  = 0; rhoIdx < rhoHelper.getRhoSize(); rhoIdx++)
      {
         int startIdx = 6 + 4 * rhoIdx;
//         double rhoValue = omega * Math.exp(omega * timeOfConstraint) * solution.get(startIdx, 0);
//         rhoValue += -omega * Math.exp(-omega * timeOfConstraint) * solution.get(startIdx + 1, 0);
         double rhoValue = (3.0 * timeOfConstraint * timeOfConstraint - 6.0 / omega2) * solution.get(startIdx + 2, 0);
         rhoValue += 2.0 * timeOfConstraint * solution.get(startIdx + 3, 0);

         assertEquals(rhoValue, rhoValueVector.get(rhoIdx), 1e-5);
         solvedVelocityAtConstraint.scaleAdd(rhoValue, rhoHelper.getBasisVector(rhoIdx), solvedVelocityAtConstraint);
      }
      solvedVelocityAtConstraint.scaleAdd(timeOfConstraint, gravityVector, solvedVelocityAtConstraint);

      EjmlUnitTests.assertEquals(taskJacobianExpected, solver.qpInputTypeA.taskJacobian, 1e-5);
      EjmlUnitTests.assertEquals(taskObjectiveExpected, solver.qpInputTypeA.taskObjective, 1e-5);

      achievedObjective.mult(taskJacobianExpected, solution);
      EjmlUnitTests.assertEquals(taskObjectiveExpected, achievedObjective, 1e-4);

      NativeMatrix solverInput_H_Expected = new NativeMatrix(taskJacobianExpected.getNumCols(), taskJacobianExpected.getNumCols());
      NativeMatrix solverInput_f_Expected = new NativeMatrix(taskJacobianExpected.getNumCols(), 1);

      solverInput_H_Expected.multTransA(taskJacobianExpected, taskJacobianExpected);
      solverInput_f_Expected.multTransA(-1.0, taskJacobianExpected, taskObjectiveExpected);

      MatrixTools.addDiagonal(solverInput_H_Expected, regularization);

      EjmlUnitTests.assertEquals(solverInput_H_Expected, solver.qpSolver.costQuadraticMatrix, 1e-10);
      EjmlUnitTests.assertEquals(solverInput_f_Expected, solver.qpSolver.quadraticCostQVector, 1e-10);

      EuclidCoreTestTools.assertTuple3DEquals(objectiveVelocity, solvedObjectiveVelocityTuple, 1e-4);
      EuclidCoreTestTools.assertTuple3DEquals(objectiveVelocity, solvedVelocityAtConstraint, 1e-4);
   }

}
