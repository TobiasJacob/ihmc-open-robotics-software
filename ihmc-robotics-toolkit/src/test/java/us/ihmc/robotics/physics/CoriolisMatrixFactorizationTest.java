package us.ihmc.robotics.physics;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.mecano.algorithms.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;

import java.util.Random;

/**
 * Tests that the following identity is valid as computed by {@link CompositeRigidBodyMassMatrixCalculator}:
 *
 * <pre>
 * (d/dt) H(q) = C(q, qDot) + C(q, qDot)<sup>T</sup>
 * </pre>
 *
 * For information on the identity, see "Numerical Methods to Compute the CoriolisMatrix and Christoffel Symbols for Rigid-BodySystems", Echeandia et. al, '20
 */
public class CoriolisMatrixFactorizationTest
{
   @Test
   public void testCoriolisMatrixFactorization()
   {
      Random random = new Random(329023);
      double epsilon = 1.0e-4;
      double dt = 1.0e-7;

      int nTests = 30;
      int maxJoints = 20;

      for (int i = 0; i < nTests; i++)
      {
         /* random floating-base serial-chain robot */
         int numberOfJoints = random.nextInt(maxJoints);
         RandomFloatingRevoluteJointChain robot = new RandomFloatingRevoluteJointChain(random, numberOfJoints);
         RigidBody elevator = robot.getElevator();
         MultiBodySystemBasics multiBodySystemInput = MultiBodySystemBasics.toMultiBodySystemBasics(elevator);

         /* set robot to random state */
         robot.nextState(random, JointStateType.CONFIGURATION, JointStateType.VELOCITY, JointStateType.ACCELERATION);

         CompositeRigidBodyMassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(multiBodySystemInput);
         massMatrixCalculator.setEnableCoriolisMatrixCalculation(true);

         /* compute mass matrix derivative through finite-difference */
         DMatrixRMaj massMatrix0 = new DMatrixRMaj(0);
         DMatrixRMaj massMatrix1 = new DMatrixRMaj(0);
         DMatrixRMaj massMatrixDot = new DMatrixRMaj(0);

         massMatrixCalculator.reset();
         massMatrix0.set(massMatrixCalculator.getMassMatrix());

         SingleRobotFirstOrderIntegrator integrator = new SingleRobotFirstOrderIntegrator(multiBodySystemInput);
         integrator.integrate(dt);
         robot.getElevator().updateFramesRecursively();

         massMatrixCalculator.reset();
         massMatrix1.set(massMatrixCalculator.getMassMatrix());

         CommonOps_DDRM.subtract(massMatrix1, massMatrix0, massMatrixDot);
         CommonOps_DDRM.scale(1.0 / dt, massMatrixDot);

         /* compute C + C^T */
         DMatrixRMaj coriolisMatrix = new DMatrixRMaj(0);
         DMatrixRMaj coriolisMatrixTranspose = new DMatrixRMaj(0);
         DMatrixRMaj coriolisSum = new DMatrixRMaj(0);

         coriolisMatrix.set(massMatrixCalculator.getCoriolisMatrix());
         coriolisMatrixTranspose.set(coriolisMatrix);
         CommonOps_DDRM.transpose(coriolisMatrixTranspose);
         CommonOps_DDRM.add(coriolisMatrix, coriolisMatrixTranspose, coriolisSum);

         /* compute error term: C + C^T - Mdot*/
         DMatrixRMaj error = new DMatrixRMaj(0);
         CommonOps_DDRM.subtract(coriolisSum, massMatrixDot, error);

         for (int j = 0; j < error.getNumElements(); j++)
         {
            Assertions.assertTrue(Math.abs(error.get(j)) < epsilon, "Invalid factorization found for serial chain with " + numberOfJoints + " one dof joints. Error term of " + error.get(i));
         }
      }
   }
}
