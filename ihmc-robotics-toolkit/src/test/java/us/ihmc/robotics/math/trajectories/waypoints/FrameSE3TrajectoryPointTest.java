package us.ihmc.robotics.math.trajectories.waypoints;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SE3TrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSE3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3WaypointBasics;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class FrameSE3TrajectoryPointTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testCommonUsageExample()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose3D(worldFrame));

      FramePoint3D poseFramePosition = new FramePoint3D(worldFrame, new Point3D(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameQuaternion poseOrientation = new FrameQuaternion(worldFrame, new AxisAngle(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      FrameSE3TrajectoryPoint frameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(worldFrame);
      SE3TrajectoryPoint simpleTrajectoryPoint = new SE3TrajectoryPoint();

      double time = 3.4;
      Point3D position = new Point3D(1.0, 2.1, 3.7);
      Quaternion orientation = new Quaternion(new Quaternion(0.1, 0.22, 0.34, 0.56));
      orientation.normalize();

      Vector3D linearVelocity = new Vector3D(-0.4, 1.2, 3.3);
      Vector3D angularVelocity = new Vector3D(1.7, 8.4, 2.2);

      simpleTrajectoryPoint.set(time, position, orientation, linearVelocity, angularVelocity);
      frameSE3TrajectoryPoint.setIncludingFrame(worldFrame, simpleTrajectoryPoint);
      frameSE3TrajectoryPoint.changeFrame(poseFrame);

      // Do some checks:
      RigidBodyTransform transformToPoseFrame = worldFrame.getTransformToDesiredFrame(poseFrame);
      transformToPoseFrame.transform(position);
      orientation.applyTransform(transformToPoseFrame);
      transformToPoseFrame.transform(linearVelocity);
      transformToPoseFrame.transform(angularVelocity);

      FrameSE3TrajectoryPoint expectedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(poseFrame);

      expectedFrameSE3TrajectoryPoint.setTime(time);
      expectedFrameSE3TrajectoryPoint.setPosition(position);
      expectedFrameSE3TrajectoryPoint.setOrientation(orientation);

      expectedFrameSE3TrajectoryPoint.setLinearVelocity(linearVelocity);
      expectedFrameSE3TrajectoryPoint.setAngularVelocity(angularVelocity);

      assertEquals(3.4, frameSE3TrajectoryPoint.getTime(), 1e-7);
      assertEquals(3.4, expectedFrameSE3TrajectoryPoint.getTime(), 1e-7);
      assertTrue(expectedFrameSE3TrajectoryPoint.epsilonEquals(frameSE3TrajectoryPoint, 1e-10));
   }

   @Test
   public void testConstructors()
   {
      double epsilon = 1.0e-20;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame aFrame = EuclidFrameRandomTools.nextReferenceFrame("aFrame", random, worldFrame);

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FramePoint3D expectedPosition = new FramePoint3D(expectedFrame);
      FrameQuaternion expectedOrientation = new FrameQuaternion(expectedFrame);
      FrameVector3D expectedLinearVelocity = new FrameVector3D(expectedFrame);
      FrameVector3D expectedAngularVelocity = new FrameVector3D(expectedFrame);

      FrameSE3TrajectoryPoint testedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

      expectedFrame = aFrame;
      expectedTime = 0.0;
      expectedPosition = new FramePoint3D(expectedFrame);
      expectedOrientation = new FrameQuaternion(expectedFrame);
      expectedLinearVelocity = new FrameVector3D(expectedFrame);
      expectedAngularVelocity = new FrameVector3D(expectedFrame);
      testedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(expectedFrame);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      testedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      FrameSE3TrajectoryPoint expectedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(expectedTime, expectedPosition, expectedOrientation,
            expectedLinearVelocity, expectedAngularVelocity);

      testedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(expectedFrameSE3TrajectoryPoint);

      assertTrue(expectedFrameSE3TrajectoryPoint.epsilonEquals(testedFrameSE3TrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedFrameSE3TrajectoryPoint.getReferenceFrame(), expectedFrameSE3TrajectoryPoint.getTime(),
            expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

      final ReferenceFrame expectedFinalFrame = aFrame;
      final double expectedFinalTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      final FramePoint3D expectedFinalPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFinalFrame, 10.0, 10.0, 10.0);
      final FrameQuaternion expectedFinalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFinalFrame);
      final FrameVector3D expectedFinalLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFinalFrame);
      final FrameVector3D expectedFinalAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFinalFrame);

      SE3TrajectoryPoint expectedSE3TrajectoryPoint = new SE3TrajectoryPoint();
      expectedSE3TrajectoryPoint.setTime(expectedFinalTime);
      expectedSE3TrajectoryPoint.setPosition(expectedFinalPosition);
      expectedSE3TrajectoryPoint.setOrientation(expectedFinalOrientation);
      expectedSE3TrajectoryPoint.setLinearVelocity(expectedFinalLinearVelocity);
      expectedSE3TrajectoryPoint.setAngularVelocity(expectedFinalAngularVelocity);

      testedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(expectedFinalFrame, expectedSE3TrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalFrame, expectedFinalTime, expectedFinalPosition, expectedFinalOrientation,
            expectedFinalLinearVelocity, expectedFinalAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

   }

   @Test
   public void testSetters()
   {
      double epsilon = 1.0e-20;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame aFrame = EuclidFrameRandomTools.nextReferenceFrame("aFrame", random, worldFrame);

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FramePoint3D expectedPosition = new FramePoint3D(expectedFrame);
      FrameQuaternion expectedOrientation = new FrameQuaternion(expectedFrame);
      FrameVector3D expectedLinearVelocity = new FrameVector3D(expectedFrame);
      FrameVector3D expectedAngularVelocity = new FrameVector3D(expectedFrame);

      final FrameSE3TrajectoryPoint testedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      testedFrameSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      testedFrameSE3TrajectoryPoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

      expectedFrame = aFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      testedFrameSE3TrajectoryPoint.setIncludingFrame(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      FrameSE3TrajectoryPoint expectedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(expectedTime, expectedPosition, expectedOrientation,
            expectedLinearVelocity, expectedAngularVelocity);

      testedFrameSE3TrajectoryPoint.setIncludingFrame(expectedFrameSE3TrajectoryPoint);

      expectedFrame = worldFrame;
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);

      expectedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity);

      testedFrameSE3TrajectoryPoint.set(expectedFrameSE3TrajectoryPoint);

      assertTrue(expectedFrameSE3TrajectoryPoint.epsilonEquals(testedFrameSE3TrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedFrameSE3TrajectoryPoint.getReferenceFrame(), expectedFrameSE3TrajectoryPoint.getTime(),
            expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

      final ReferenceFrame expectedFinalFrame = aFrame;
      final double expectedFinalTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      final FramePoint3D expectedFinalPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFinalFrame, 10.0, 10.0, 10.0);
      final FrameQuaternion expectedFinalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFinalFrame);
      final FrameVector3D expectedFinalLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFinalFrame);
      final FrameVector3D expectedFinalAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFinalFrame);

      SE3TrajectoryPoint expectedSE3TrajectoryPoint = new SE3TrajectoryPoint();
      expectedSE3TrajectoryPoint.setTime(expectedFinalTime);
      expectedSE3TrajectoryPoint.setPosition(expectedFinalPosition);
      expectedSE3TrajectoryPoint.setOrientation(expectedFinalOrientation);
      expectedSE3TrajectoryPoint.setLinearVelocity(expectedFinalLinearVelocity);
      expectedSE3TrajectoryPoint.setAngularVelocity(expectedFinalAngularVelocity);

      testedFrameSE3TrajectoryPoint.setIncludingFrame(expectedFinalFrame, expectedSE3TrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalFrame, expectedFinalTime, expectedFinalPosition, expectedFinalOrientation,
            expectedFinalLinearVelocity, expectedFinalAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

   }

   @Test
   public void testChangeFrame() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      FramePoint3D expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameQuaternion expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      FrameVector3D expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      FrameVector3D expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      FrameSE3TrajectoryPoint testedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(expectedTime, expectedPosition, expectedOrientation,
            expectedLinearVelocity, expectedAngularVelocity);

      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = EuclidFrameRandomTools.nextReferenceFrame("randomFrame" + i, random, random.nextBoolean() ? worldFrame : expectedFrame);

         expectedPosition.changeFrame(expectedFrame);
         expectedOrientation.changeFrame(expectedFrame);
         expectedLinearVelocity.changeFrame(expectedFrame);
         expectedAngularVelocity.changeFrame(expectedFrame);
         testedFrameSE3TrajectoryPoint.changeFrame(expectedFrame);

         assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
               expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      FramePoint3D expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameQuaternion expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      FrameVector3D expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      FrameVector3D expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      FrameSE3TrajectoryPoint testedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(expectedTime, expectedPosition, expectedOrientation,
            expectedLinearVelocity, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero();
      expectedOrientation.setToZero();
      expectedLinearVelocity.setToZero();
      expectedAngularVelocity.setToZero();
      testedFrameSE3TrajectoryPoint.setToZero();

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);

      expectedFrame = EuclidFrameRandomTools.nextReferenceFrame("blop", random, worldFrame);
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 10.0, 10.0, 10.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);
      testedFrameSE3TrajectoryPoint.setIncludingFrame(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero(expectedFrame);
      expectedOrientation.setToZero(expectedFrame);
      expectedLinearVelocity.setToZero(expectedFrame);
      expectedAngularVelocity.setToZero(expectedFrame);
      testedFrameSE3TrajectoryPoint.setToZero(expectedFrame);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity, testedFrameSE3TrajectoryPoint, epsilon);
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      FramePoint3D expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameQuaternion expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, expectedFrame);
      FrameVector3D expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      FrameVector3D expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, expectedFrame);
      FrameSE3TrajectoryPoint testedFrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(expectedTime, expectedPosition, expectedOrientation,
            expectedLinearVelocity, expectedAngularVelocity);

      testedFrameSE3TrajectoryPoint.setToNaN();
      assertTrue(Double.isNaN(testedFrameSE3TrajectoryPoint.getTime()));
      assertTrue(testedFrameSE3TrajectoryPoint.containsNaN());

      expectedFrame = EuclidFrameRandomTools.nextReferenceFrame("blop", random, worldFrame);
      expectedTime = RandomNumbers.nextDouble(random, 0.0, 1000.0);
      expectedPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 10.0, 10.0, 10.0);
      expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      expectedLinearVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);
      expectedAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);
      testedFrameSE3TrajectoryPoint.setIncludingFrame(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      testedFrameSE3TrajectoryPoint.setToNaN(expectedFrame);

      assertTrue(expectedFrame == testedFrameSE3TrajectoryPoint.getReferenceFrame());
      assertTrue(Double.isNaN(testedFrameSE3TrajectoryPoint.getTime()));
      assertTrue(testedFrameSE3TrajectoryPoint.containsNaN());
   }

   static void assertTrajectoryPointContainsExpectedData(ReferenceFrame expectedFrame, double expectedTime, FramePoint3DReadOnly expectedPosition,
         FrameQuaternionReadOnly expectedOrientation, FrameVector3DReadOnly expectedLinearVelocity, FrameVector3DReadOnly expectedAngularVelocity,
         FrameSE3TrajectoryPoint testedFrameSE3TrajectoryPoint, double epsilon)
   {
      assertTrue(expectedFrame == testedFrameSE3TrajectoryPoint.getReferenceFrame());
      assertEquals(expectedTime, testedFrameSE3TrajectoryPoint.getTime(), epsilon);
      assertTrue(expectedPosition.epsilonEquals(testedFrameSE3TrajectoryPoint.getPosition(), epsilon));
      assertTrue(expectedOrientation.geometricallyEquals(testedFrameSE3TrajectoryPoint.getOrientation(), epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(testedFrameSE3TrajectoryPoint.getLinearVelocity(), epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(testedFrameSE3TrajectoryPoint.getAngularVelocity(), epsilon));

      Point3D actualPosition = new Point3D();
      Quaternion actualOrientation = new Quaternion();
      Vector3D actualLinearVelocity = new Vector3D();
      Vector3D actualAngularVelocity = new Vector3D();

      testedFrameSE3TrajectoryPoint.getPosition(actualPosition);
      testedFrameSE3TrajectoryPoint.getOrientation(actualOrientation);
      testedFrameSE3TrajectoryPoint.getLinearVelocity(actualLinearVelocity);
      testedFrameSE3TrajectoryPoint.getAngularVelocity(actualAngularVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualPosition, epsilon));
      assertTrue(expectedOrientation.geometricallyEquals(actualOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualAngularVelocity, epsilon));

      FramePoint3D actualFramePosition = new FramePoint3D();
      FrameQuaternion actualFrameOrientation = new FrameQuaternion();
      FrameVector3D actualFrameLinearVelocity = new FrameVector3D();
      FrameVector3D actualFrameAngularVelocity = new FrameVector3D();

      testedFrameSE3TrajectoryPoint.getPositionIncludingFrame(actualFramePosition);
      testedFrameSE3TrajectoryPoint.getOrientationIncludingFrame(actualFrameOrientation);
      testedFrameSE3TrajectoryPoint.getLinearVelocityIncludingFrame(actualFrameLinearVelocity);
      testedFrameSE3TrajectoryPoint.getAngularVelocityIncludingFrame(actualFrameAngularVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedOrientation.geometricallyEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));

      actualFramePosition = new FramePoint3D(expectedFrame);
      actualFrameOrientation = new FrameQuaternion(expectedFrame);
      actualFrameLinearVelocity = new FrameVector3D(expectedFrame);
      actualFrameAngularVelocity = new FrameVector3D(expectedFrame);

      testedFrameSE3TrajectoryPoint.getPosition(actualFramePosition);
      testedFrameSE3TrajectoryPoint.getOrientation(actualFrameOrientation);
      testedFrameSE3TrajectoryPoint.getLinearVelocity(actualFrameLinearVelocity);
      testedFrameSE3TrajectoryPoint.getAngularVelocity(actualFrameAngularVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedOrientation.geometricallyEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));
   }

   @Test
   public void testSomeSetsAngGets()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FrameSE3TrajectoryPoint FrameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(worldFrame);
      FrameSE3TrajectoryPoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      SE3TrajectoryPoint simpleTrajectoryPoint = new SE3TrajectoryPoint();

      double time = 3.4;
      Point3D position = new Point3D(1.0, 2.1, 3.7);
      Quaternion orientation = new Quaternion(new Quaternion(0.1, 0.22, 0.34, 0.56));
      orientation.normalize();

      Vector3D linearVelocity = new Vector3D(-0.4, 1.2, 3.3);
      Vector3D angularVelocity = new Vector3D(1.7, 8.4, 2.2);

      simpleTrajectoryPoint.set(time, position, orientation, linearVelocity, angularVelocity);
      FrameSE3TrajectoryPoint.setIncludingFrame(worldFrame, simpleTrajectoryPoint);

      // Check some get calls:
      FramePoint3D pointForVerification = new FramePoint3D(worldFrame);
      FrameQuaternion quaternionForVerification = new FrameQuaternion(worldFrame);
      FrameVector3D linearVelocityForVerification = new FrameVector3D(worldFrame);
      FrameVector3D angularVelocityForVerification = new FrameVector3D(worldFrame);

      FrameSE3TrajectoryPoint.getPosition(pointForVerification);
      FrameSE3TrajectoryPoint.getOrientation(quaternionForVerification);
      FrameSE3TrajectoryPoint.getLinearVelocity(linearVelocityForVerification);
      FrameSE3TrajectoryPoint.getAngularVelocity(angularVelocityForVerification);

      assertEquals(time, FrameSE3TrajectoryPoint.getTime(), 1e-10);
      assertTrue(pointForVerification.epsilonEquals(position, 1e-10));
      assertTrue(quaternionForVerification.epsilonEquals(orientation, 1e-10));
      assertTrue(linearVelocityForVerification.epsilonEquals(linearVelocity, 1e-10));
      assertTrue(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-10));

      // Check NaN calls:
      assertFalse(FrameSE3TrajectoryPoint.containsNaN());
      FrameSE3TrajectoryPoint.setPositionToNaN();
      assertTrue(FrameSE3TrajectoryPoint.containsNaN());
      FrameSE3TrajectoryPoint.setPositionToZero();

      assertFalse(FrameSE3TrajectoryPoint.containsNaN());
      FrameSE3TrajectoryPoint.setOrientationToNaN();
      assertTrue(FrameSE3TrajectoryPoint.containsNaN());
      FrameSE3TrajectoryPoint.setOrientationToZero();

      assertFalse(FrameSE3TrajectoryPoint.containsNaN());
      FrameSE3TrajectoryPoint.setLinearVelocityToNaN();
      assertTrue(FrameSE3TrajectoryPoint.containsNaN());
      FrameSE3TrajectoryPoint.setLinearVelocityToZero();

      assertFalse(FrameSE3TrajectoryPoint.containsNaN());
      FrameSE3TrajectoryPoint.setAngularVelocityToNaN();
      assertTrue(FrameSE3TrajectoryPoint.containsNaN());
      FrameSE3TrajectoryPoint.setAngularVelocityToZero();
      assertFalse(FrameSE3TrajectoryPoint.containsNaN());

      FrameSE3TrajectoryPoint.getPosition(position);
      FrameSE3TrajectoryPoint.getOrientation(orientation);
      FrameSE3TrajectoryPoint.getLinearVelocity(linearVelocity);
      FrameSE3TrajectoryPoint.getAngularVelocity(angularVelocity);

      // Make sure they are all equal to zero:
      assertTrue(position.epsilonEquals(new Point3D(), 1e-10));
      assertTrue(orientation.epsilonEquals(new Quaternion(), 1e-10));
      assertTrue(linearVelocity.epsilonEquals(new Vector3D(), 1e-10));
      assertTrue(angularVelocity.epsilonEquals(new Vector3D(), 1e-10));

      time = 9.9;
      pointForVerification.set(3.9, 2.2, 1.1);
      quaternionForVerification.setYawPitchRoll(0.2, 0.6, 1.1);
      linearVelocityForVerification.set(8.8, 1.4, 9.22);
      angularVelocityForVerification.set(7.1, 2.2, 3.33);

      assertFalse(Math.abs(FrameSE3TrajectoryPoint.getTime() - time) < 1e-7);
      assertFalse(pointForVerification.epsilonEquals(position, 1e-7));
      assertFalse(quaternionForVerification.epsilonEquals(orientation, 1e-7));
      assertFalse(linearVelocityForVerification.epsilonEquals(linearVelocity, 1e-7));
      assertFalse(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-7));

      FrameSE3TrajectoryPoint.set(time, pointForVerification, quaternionForVerification, linearVelocityForVerification, angularVelocityForVerification);

      FrameSE3TrajectoryPoint.getPosition(position);
      FrameSE3TrajectoryPoint.getOrientation(orientation);
      FrameSE3TrajectoryPoint.getLinearVelocity(linearVelocity);
      FrameSE3TrajectoryPoint.getAngularVelocity(angularVelocity);

      assertEquals(time, FrameSE3TrajectoryPoint.getTime(), 1e-10);
      assertTrue(pointForVerification.epsilonEquals(position, 1e-10));
      assertTrue(quaternionForVerification.epsilonEquals(orientation, 1e-10));
      assertTrue(linearVelocityForVerification.epsilonEquals(linearVelocity, 1e-10));
      assertTrue(angularVelocityForVerification.epsilonEquals(angularVelocity, 1e-10));

      FrameSE3TrajectoryPoint FrameSE3TrajectoryPointTwo = new FrameSE3TrajectoryPoint(worldFrame);

      double positionDistance = FrameSE3TrajectoryPoint.positionDistance(FrameSE3TrajectoryPointTwo);
      assertEquals(4.610856753359402, positionDistance, 1e-7);
      assertFalse(FrameSE3TrajectoryPoint.epsilonEquals(FrameSE3TrajectoryPointTwo, 1e-7));

      FrameSE3TrajectoryPointTwo.set(FrameSE3TrajectoryPoint);
      positionDistance = FrameSE3TrajectoryPoint.positionDistance(FrameSE3TrajectoryPointTwo);
      assertEquals(0.0, positionDistance, 1e-7);
      assertTrue(FrameSE3TrajectoryPoint.epsilonEquals(FrameSE3TrajectoryPointTwo, 1e-7));

      SE3TrajectoryPoint simplePoint = new SE3TrajectoryPoint();
      FrameSE3TrajectoryPoint.get(simplePoint);

      FrameSE3TrajectoryPoint.setToNaN();
      assertTrue(FrameSE3TrajectoryPoint.containsNaN());
      positionDistance = FrameSE3TrajectoryPoint.positionDistance(FrameSE3TrajectoryPointTwo);
      assertTrue(Double.isNaN(positionDistance));
      assertFalse(FrameSE3TrajectoryPoint.epsilonEquals(FrameSE3TrajectoryPointTwo, 1e-7));

      SE3TrajectoryPointBasics trajectoryPointAsInterface = simplePoint;
      FrameSE3TrajectoryPoint.set(trajectoryPointAsInterface);

      positionDistance = FrameSE3TrajectoryPoint.positionDistance(FrameSE3TrajectoryPointTwo);
      assertEquals(0.0, positionDistance, 1e-7);
      assertTrue(FrameSE3TrajectoryPoint.epsilonEquals(FrameSE3TrajectoryPointTwo, 1e-7));
   }

   @Test
   public void testSomeMoreSettersAndGetters()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FrameSE3TrajectoryPoint frameSE3TrajectoryPoint = new FrameSE3TrajectoryPoint(worldFrame);
      frameSE3TrajectoryPoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      double time = 3.4;
      FramePoint3D position = new FramePoint3D(worldFrame, 1.0, 2.1, 3.7);
      FrameQuaternion orientation = new FrameQuaternion(worldFrame, new Quaternion(0.1, 0.22, 0.34, 0.56));

      FrameVector3D linearVelocity = new FrameVector3D(worldFrame, -0.4, 1.2, 3.3);
      FrameVector3D angularVelocity = new FrameVector3D(worldFrame, 1.7, 8.4, 2.2);

      frameSE3TrajectoryPoint.setTime(time);
      frameSE3TrajectoryPoint.setPosition(position);
      frameSE3TrajectoryPoint.setOrientation(orientation);
      frameSE3TrajectoryPoint.setLinearVelocity(linearVelocity);
      frameSE3TrajectoryPoint.setAngularVelocity(angularVelocity);

      PoseReferenceFrame poseFrame = new PoseReferenceFrame("poseFrame", new FramePose3D(worldFrame));

      FramePoint3D poseFramePosition = new FramePoint3D(worldFrame, new Point3D(0.5, 7.7, 9.2));
      poseFrame.setPositionAndUpdate(poseFramePosition);

      FrameQuaternion poseOrientation = new FrameQuaternion(worldFrame, new AxisAngle(1.2, 3.9, 4.7, 2.2));
      poseFrame.setOrientationAndUpdate(poseOrientation);

      frameSE3TrajectoryPoint.changeFrame(poseFrame);

      assertFalse(position.epsilonEquals(frameSE3TrajectoryPoint.getPositionCopy(), 1e-10));
      assertFalse(orientation.epsilonEquals(frameSE3TrajectoryPoint.getOrientationCopy(), 1e-10));
      assertFalse(linearVelocity.epsilonEquals(frameSE3TrajectoryPoint.getLinearVelocityCopy(), 1e-10));
      assertFalse(angularVelocity.epsilonEquals(frameSE3TrajectoryPoint.getAngularVelocityCopy(), 1e-10));

      position.changeFrame(poseFrame);
      orientation.changeFrame(poseFrame);
      linearVelocity.changeFrame(poseFrame);
      angularVelocity.changeFrame(poseFrame);

      assertTrue(position.epsilonEquals(frameSE3TrajectoryPoint.getPositionCopy(), 1e-10));
      assertTrue(orientation.epsilonEquals(frameSE3TrajectoryPoint.getOrientationCopy(), 1e-10));
      assertTrue(linearVelocity.epsilonEquals(frameSE3TrajectoryPoint.getLinearVelocityCopy(), 1e-10));
      assertTrue(angularVelocity.epsilonEquals(frameSE3TrajectoryPoint.getAngularVelocityCopy(), 1e-10));

      FrameSE3TrajectoryPoint frameSE3TrajectoryPointTwo = new FrameSE3TrajectoryPoint(poseFrame);
      frameSE3TrajectoryPointTwo.setTime(time);
      frameSE3TrajectoryPointTwo.setPosition(position);
      frameSE3TrajectoryPointTwo.setOrientation(orientation);
      frameSE3TrajectoryPointTwo.setLinearVelocity(linearVelocity);
      frameSE3TrajectoryPointTwo.setAngularVelocity(angularVelocity);
      assertTrue(frameSE3TrajectoryPointTwo.epsilonEquals(frameSE3TrajectoryPoint, 1e-10));

      frameSE3TrajectoryPointTwo = new FrameSE3TrajectoryPoint(worldFrame);
      frameSE3TrajectoryPointTwo.setIncludingFrame(poseFrame, time, new Point3D(position), orientation, new Vector3D(linearVelocity),
                                                   new Vector3D(angularVelocity));
      assertTrue(frameSE3TrajectoryPointTwo.epsilonEquals(frameSE3TrajectoryPoint, 1e-10));

      frameSE3TrajectoryPointTwo = new FrameSE3TrajectoryPoint(poseFrame);
      SE3WaypointBasics se3Waypoint = frameSE3TrajectoryPoint;
      frameSE3TrajectoryPointTwo.set(time, se3Waypoint);
      assertTrue(frameSE3TrajectoryPointTwo.epsilonEquals(frameSE3TrajectoryPoint, 1e-10));

      frameSE3TrajectoryPointTwo = new FrameSE3TrajectoryPoint(worldFrame);
      frameSE3TrajectoryPointTwo.setIncludingFrame(poseFrame, time, se3Waypoint);
      assertTrue(frameSE3TrajectoryPointTwo.epsilonEquals(frameSE3TrajectoryPoint, 1e-10));

      frameSE3TrajectoryPointTwo = new FrameSE3TrajectoryPoint(poseFrame);
      FrameSE3WaypointBasics frameSE3Waypoint = frameSE3TrajectoryPoint;
      frameSE3TrajectoryPointTwo.set(time, frameSE3Waypoint);
      assertTrue(frameSE3TrajectoryPointTwo.epsilonEquals(frameSE3TrajectoryPoint, 1e-10));

      frameSE3TrajectoryPointTwo = new FrameSE3TrajectoryPoint(worldFrame);
      frameSE3TrajectoryPointTwo.setIncludingFrame(time, frameSE3Waypoint);
      assertTrue(frameSE3TrajectoryPointTwo.epsilonEquals(frameSE3TrajectoryPoint, 1e-10));

   }
}
