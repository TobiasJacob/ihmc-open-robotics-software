package us.ihmc.valkyrie;

import java.util.List;

import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;

public class ValkyrieInitialSetup implements DRCRobotInitialSetup<HumanoidFloatingRootJointRobot>
{
   private double groundZ;
   private double initialYaw;
   private final RigidBodyTransform rootToWorld = new RigidBodyTransform();
   private final Vector3D positionInWorld = new Vector3D();
   private final Vector3D offset = new Vector3D();
   private final Quaternion rotation = new Quaternion();

   public ValkyrieInitialSetup()
   {
   }

   public ValkyrieInitialSetup(double groundZ, double initialYaw)
   {
      setInitialGroundHeight(groundZ);
      setInitialYaw(initialYaw);
   }

   public List<Double> getInitialJointAngles() // Implement for kinematics sim & start pose
   {
      return null;
   }

   public Pose3DReadOnly getInitialPelvisPose() // Implement for kinematics sim & start pose
   {
      return null;
   }

   @Override
   public void initializeRobot(HumanoidFloatingRootJointRobot robot, HumanoidJointNameMap jointMap)
   {
      setActuatorPositions(robot, jointMap);
      positionRobotInWorld(robot);
   }

   private void setActuatorPositions(FloatingRootJointRobot robot, HumanoidJointNameMap jointMap)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String hipPitch = jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH);
         String knee = jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH);
         String anklePitch = jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH);
         String hipRoll = jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL);
         String ankleRoll = jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL);

         robot.getOneDegreeOfFreedomJoint(hipPitch).setQ(-0.6);
         robot.getOneDegreeOfFreedomJoint(knee).setQ(1.3);
         robot.getOneDegreeOfFreedomJoint(anklePitch).setQ(-0.70);
         robot.getOneDegreeOfFreedomJoint(hipRoll).setQ(0.0);
         robot.getOneDegreeOfFreedomJoint(ankleRoll).setQ(0.0);

         String shoulderRoll = jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL);
         String shoulderPitch = jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH);
         String elbowPitch = jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH);
         String elbowRoll = jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL);

         if (shoulderRoll != null)
            robot.getOneDegreeOfFreedomJoint(shoulderRoll).setQ(robotSide.negateIfRightSide(-1.2));//inv
         if (shoulderPitch != null)
            robot.getOneDegreeOfFreedomJoint(shoulderPitch).setQ(-0.2);
         if (elbowPitch != null)
            robot.getOneDegreeOfFreedomJoint(elbowPitch).setQ(robotSide.negateIfRightSide(-1.5));//inv
         if (elbowRoll != null)
            robot.getOneDegreeOfFreedomJoint(elbowRoll).setQ(1.3);
      }
      robot.update();
   }

   private void positionRobotInWorld(HumanoidFloatingRootJointRobot robot)
   {
      robot.getRootJoint().setPosition(0.0, 0.0, 0.0);
      robot.update();
      robot.getRootJointToWorldTransform(rootToWorld);
      rootToWorld.get(rotation, positionInWorld);
      positionInWorld.add(offset);
      positionInWorld.addZ(groundZ - getLowestFootContactPointHeight(robot));
      robot.setPositionInWorld(positionInWorld);

      FrameQuaternion frameOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame(), rotation);
      YawPitchRoll yawPitchRoll = new YawPitchRoll(frameOrientation);
      yawPitchRoll.setYaw(initialYaw);
      frameOrientation.set(yawPitchRoll);

      robot.setOrientation(frameOrientation);
      robot.update();
   }

   private double getLowestFootContactPointHeight(HumanoidFloatingRootJointRobot robot)
   {
      List<GroundContactPoint> contactPoints = robot.getFootGroundContactPoints(RobotSide.LEFT);
      double height = Double.POSITIVE_INFINITY;

      if (contactPoints.size() == 0)
      {
         height = -1.0050100629487357;
      }
      else
      {
         for (GroundContactPoint gc : contactPoints)
         {
            height = Math.min(height, gc.getPositionCopy().getZ());
         }
      }

      return height;
   }

   public void getOffset(Vector3D offsetToPack)
   {
      offsetToPack.set(offset);
   }

   public void setOffset(Vector3D offset)
   {
      this.offset.set(offset);
   }

   @Override
   public void setInitialYaw(double yaw)
   {
      initialYaw = yaw;
   }

   @Override
   public void setInitialGroundHeight(double groundHeight)
   {
      groundZ = groundHeight;
   }

   @Override
   public double getInitialYaw()
   {
      return initialYaw;
   }

   @Override
   public double getInitialGroundHeight()
   {
      return groundZ;
   }
}
