package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import java.util.Set;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.robotics.screwTheory.InvertedFourBarJoint;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.IMUMount;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SimulatedSensorHolderAndReaderFromRobotFactory implements SensorReaderFactory
{
   private final YoRegistry registry = new YoRegistry("SensorReaderFactory");
   private final Robot robot;

   private final ArrayList<IMUMount> imuMounts = new ArrayList<IMUMount>();
   private final ArrayList<WrenchCalculatorInterface> groundContactPointBasedWrenchCalculators = new ArrayList<WrenchCalculatorInterface>();

   private SimulatedSensorHolderAndReader simulatedSensorHolderAndReader;
   private StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions;
   private final SensorProcessingConfiguration sensorProcessingConfiguration;

   public SimulatedSensorHolderAndReaderFromRobotFactory(Robot robot, SensorProcessingConfiguration sensorProcessingConfiguration)
   {
      this.robot = robot;
      this.sensorProcessingConfiguration = sensorProcessingConfiguration;

      robot.getIMUMounts(imuMounts);
      robot.getForceSensors(groundContactPointBasedWrenchCalculators);
   }

   @Override
   public void build(FloatingJointBasics rootJoint, IMUDefinition[] imuDefinitions, ForceSensorDefinition[] forceSensorDefinitions,
                     JointDesiredOutputListBasics estimatorDesiredJointDataHolder, YoRegistry parentRegistry)
   {
      List<Joint> rootJoints = robot.getRootJoints();

      if (rootJoints.size() > 1)
      {
         throw new RuntimeException("Robot has more than 1 rootJoint");
      }

      final Joint scsRootJoint = rootJoints.get(0);
      if (!(scsRootJoint instanceof FloatingJoint))
         throw new RuntimeException("Not FloatingJoint rootjoint found");

      stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();
      SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootJoint.getPredecessor()).forEach(stateEstimatorSensorDefinitions::addJointSensorDefinition);
      for (IMUDefinition imuDefinition : imuDefinitions)
      {
         stateEstimatorSensorDefinitions.addIMUSensorDefinition(imuDefinition);
      }
      for (ForceSensorDefinition forceSensorDefinition : forceSensorDefinitions)
      {
         stateEstimatorSensorDefinitions.addForceSensorDefinition(forceSensorDefinition);
      }

      this.simulatedSensorHolderAndReader = new SimulatedSensorHolderAndReader(stateEstimatorSensorDefinitions,
                                                                               sensorProcessingConfiguration,
                                                                               robot.getYoTime(),
                                                                               registry);

      SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootJoint.getPredecessor()).forEach(joint ->
      {
         final DoubleProvider position, velocity, tau;

         if (joint instanceof InvertedFourBarJoint)
         {
            InvertedFourBarJoint fourBarJoint = (InvertedFourBarJoint) joint;
            OneDegreeOfFreedomJoint scsJointA = (OneDegreeOfFreedomJoint) robot.getJoint(fourBarJoint.getJointA().getName());
            OneDegreeOfFreedomJoint scsJointB = (OneDegreeOfFreedomJoint) robot.getJoint(fourBarJoint.getJointB().getName());
            OneDegreeOfFreedomJoint scsJointC = (OneDegreeOfFreedomJoint) robot.getJoint(fourBarJoint.getJointC().getName());
            OneDegreeOfFreedomJoint scsJointD = (OneDegreeOfFreedomJoint) robot.getJoint(fourBarJoint.getJointD().getName());

            if (scsJointA != null && scsJointD != null)
            {
               position = () -> scsJointA.getQ() + scsJointD.getQ();
               velocity = () -> scsJointA.getQD() + scsJointD.getQD();
               tau = () ->
               {
                  // This is inaccurate as the Jacobian should be updated with the most recent position, but it shouldn't matter.
                  DMatrixRMaj loopJacobian = fourBarJoint.getFourBarFunction().getLoopJacobian();
                  return loopJacobian.get(0) * scsJointA.getTau() + loopJacobian.get(3) * scsJointD.getTau();
               };
            }
            else
            {
               position = () -> scsJointB.getQ() + scsJointC.getQ();
               velocity = () -> scsJointB.getQD() + scsJointC.getQD();
               tau = () ->
               {
                  // This is inaccurate as the Jacobian should be updated with the most recent position, but it shouldn't matter.
                  DMatrixRMaj loopJacobian = fourBarJoint.getFourBarFunction().getLoopJacobian();
                  return loopJacobian.get(1) * scsJointB.getTau() + loopJacobian.get(2) * scsJointC.getTau();
               };
            }
         }
         else
         {
            OneDegreeOfFreedomJoint scsJoint = (OneDegreeOfFreedomJoint) robot.getJoint(joint.getName());
            position = scsJoint.getQYoVariable();
            velocity = scsJoint.getQDYoVariable();
            tau = scsJoint.getTauYoVariable();
         }

         simulatedSensorHolderAndReader.addJointPositionSensorPort(joint, position);
         simulatedSensorHolderAndReader.addJointVelocitySensorPort(joint, velocity);
         simulatedSensorHolderAndReader.addJointTorqueSensorPort(joint, tau);
      });

      Map<IMUMount, IMUDefinition> imuSensorMap = Stream.of(imuDefinitions).collect(Collectors.toMap(def -> findIMUMount(def.getName()), Function.identity()));
      Map<WrenchCalculatorInterface, ForceSensorDefinition> wrenchSensorMap = Stream.of(forceSensorDefinitions)
                                                                                    .collect(Collectors.toMap(def -> findWrenchSensor(def.getSensorName()),
                                                                                                              Function.identity()));
      createAndAddOrientationSensors(imuSensorMap, registry);
      createAndAddAngularVelocitySensors(imuSensorMap, registry);
      createAndAddLinearAccelerationSensors(imuSensorMap, registry);
      createAndAddForceSensors(wrenchSensorMap, registry);

      parentRegistry.addChild(registry);
   }

   private IMUMount findIMUMount(String sensorName)
   {
      return imuMounts.stream().filter(candidate -> candidate.getName().equals(sensorName)).findFirst().get();
   }

   private WrenchCalculatorInterface findWrenchSensor(String sensorName)
   {
      return groundContactPointBasedWrenchCalculators.stream().filter(candidate -> candidate.getName().equals(sensorName)).findFirst().get();
   }

   private void createAndAddForceSensors(Map<WrenchCalculatorInterface, ForceSensorDefinition> forceSensorDefinitions2, YoRegistry registry)
   {
      for (Entry<WrenchCalculatorInterface, ForceSensorDefinition> forceSensorDefinitionEntry : forceSensorDefinitions2.entrySet())
      {
         WrenchCalculatorInterface groundContactPointBasedWrenchCalculator = forceSensorDefinitionEntry.getKey();
         simulatedSensorHolderAndReader.addForceTorqueSensorPort(forceSensorDefinitionEntry.getValue(), groundContactPointBasedWrenchCalculator);
      }
   }

   @Override
   public SimulatedSensorHolderAndReader getSensorReader()
   {
      return simulatedSensorHolderAndReader;
   }

   private void createAndAddOrientationSensors(Map<IMUMount, IMUDefinition> imuDefinitions, YoRegistry registry)
   {
      Set<IMUMount> imuMounts = imuDefinitions.keySet();

      for (IMUMount imuMount : imuMounts)
      {
         IMUDefinition imuDefinition = imuDefinitions.get(imuMount);

         simulatedSensorHolderAndReader.addOrientationSensorPort(imuDefinition, new QuaternionProvider()
         {
            private final Quaternion orientation = new Quaternion();

            @Override
            public QuaternionReadOnly getValue()
            {
               imuMount.getOrientation(orientation);
               return orientation;
            }
         });
      }
   }

   private void createAndAddAngularVelocitySensors(Map<IMUMount, IMUDefinition> imuDefinitions, YoRegistry registry)
   {
      Set<IMUMount> imuMounts = imuDefinitions.keySet();

      for (IMUMount imuMount : imuMounts)
      {
         IMUDefinition imuDefinition = imuDefinitions.get(imuMount);

         simulatedSensorHolderAndReader.addAngularVelocitySensorPort(imuDefinition, new Vector3DProvider()
         {
            private final Vector3D angularVelocity = new Vector3D();

            @Override
            public Vector3DReadOnly getValue()
            {
               imuMount.getAngularVelocityInBody(angularVelocity);
               return angularVelocity;
            }
         });
      }
   }

   private void createAndAddLinearAccelerationSensors(Map<IMUMount, IMUDefinition> imuDefinitions, YoRegistry registry)
   {
      Set<IMUMount> imuMounts = imuDefinitions.keySet();

      for (IMUMount imuMount : imuMounts)
      {
         IMUDefinition imuDefinition = imuDefinitions.get(imuMount);

         simulatedSensorHolderAndReader.addLinearAccelerationSensorPort(imuDefinition, new Vector3DProvider()
         {
            private final Vector3D linearAcceleration = new Vector3D();

            @Override
            public Vector3DReadOnly getValue()
            {
               imuMount.getLinearAccelerationInBody(linearAcceleration);
               return linearAcceleration;
            }
         });
      }
   }

   @Override
   public StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions()
   {
      return stateEstimatorSensorDefinitions;
   }

   @Override
   public boolean useStateEstimator()
   {
      return true;
   }
}
