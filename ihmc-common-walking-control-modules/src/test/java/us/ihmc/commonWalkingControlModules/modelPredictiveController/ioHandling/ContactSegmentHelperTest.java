package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.robotics.math.trajectories.core.Polynomial3D;

import java.util.Random;

public class ContactSegmentHelperTest
{
   private static final int iters = 1000;
   private static final double positionEpsilon = 5e-5;
   private static final double velocityEpsilon = 5e-3;

   @Test
   public void testRandomCubicTrajectory()
   {
      Random random = new Random(1738L);
      ContactSegmentHelper helper = new ContactSegmentHelper();
      for (int i = 0; i < iters; i++)
      {
         Polynomial3D trajectory = new Polynomial3D(4);

         FramePoint3D startPoint = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D endPoint = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D startVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D endVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         SettableContactStateProvider segmentToInterpolateFromStart = new SettableContactStateProvider();
         SettableContactStateProvider segmentToInterpolateFromEnd = new SettableContactStateProvider();

         segmentToInterpolateFromStart.setStartECMPPosition(startPoint);
         segmentToInterpolateFromStart.setEndECMPPosition(endPoint);
         segmentToInterpolateFromStart.setStartECMPVelocity(startVelocity);
         segmentToInterpolateFromStart.setEndECMPVelocity(endVelocity);

         segmentToInterpolateFromEnd.setStartECMPPosition(startPoint);
         segmentToInterpolateFromEnd.setEndECMPPosition(endPoint);
         segmentToInterpolateFromEnd.setStartECMPVelocity(startVelocity);
         segmentToInterpolateFromEnd.setEndECMPVelocity(endVelocity);

         double startTime = RandomNumbers.nextDouble(random, 10.0);
         double duration = RandomNumbers.nextDouble(random, 0.0, 10.0);
         double endTime = startTime + duration;
         double timeFromStart = RandomNumbers.nextDouble(random, startTime, endTime);
         double timeFromEnd = RandomNumbers.nextDouble(random, startTime, endTime);

         double alphaFromStart = (timeFromStart - startTime) / duration;
         double alphaFromEnd = (timeFromEnd - startTime) / duration;

         trajectory.setCubic(startTime, endTime, startPoint, startVelocity, endPoint, endVelocity);

         segmentToInterpolateFromStart.getTimeInterval().setInterval(startTime, endTime);
         segmentToInterpolateFromEnd.getTimeInterval().setInterval(startTime, endTime);

         helper.cubicInterpolateStartOfSegment(segmentToInterpolateFromStart, alphaFromStart);
         helper.cubicInterpolateEndOfSegment(segmentToInterpolateFromEnd, alphaFromEnd);

         trajectory.compute(timeFromStart);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(trajectory.getPosition(), segmentToInterpolateFromStart.getECMPStartPosition(), positionEpsilon);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(trajectory.getVelocity(), segmentToInterpolateFromStart.getECMPStartVelocity(), velocityEpsilon);

         trajectory.compute(timeFromEnd);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(trajectory.getPosition(), segmentToInterpolateFromEnd.getECMPEndPosition(), positionEpsilon);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(trajectory.getVelocity(), segmentToInterpolateFromEnd.getECMPEndVelocity(), velocityEpsilon);
      }
   }

   @Test
   public void testRandomLinearTrajectory()
   {
      Random random = new Random(1738L);
      ContactSegmentHelper helper = new ContactSegmentHelper();
      for (int i = 0; i < iters; i++)
      {

         FramePoint3D startPoint = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D endPoint = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());

         SettableContactStateProvider segmentToInterpolateFromStart = new SettableContactStateProvider();
         SettableContactStateProvider segmentToInterpolateFromEnd = new SettableContactStateProvider();

         segmentToInterpolateFromStart.setStartECMPPosition(startPoint);
         segmentToInterpolateFromStart.setEndECMPPosition(endPoint);

         segmentToInterpolateFromEnd.setStartECMPPosition(startPoint);
         segmentToInterpolateFromEnd.setEndECMPPosition(endPoint);

         double startTime = RandomNumbers.nextDouble(random, 10.0);
         double duration = RandomNumbers.nextDouble(random, 0.0, 10.0);
         double endTime = startTime + duration;
         double timeFromStart = RandomNumbers.nextDouble(random, startTime, endTime);
         double timeFromEnd = RandomNumbers.nextDouble(random, startTime, endTime);

         double alphaFromStart = (timeFromStart - startTime) / duration;
         double alphaFromEnd = (timeFromEnd - startTime) / duration;

         segmentToInterpolateFromStart.getTimeInterval().setInterval(startTime, endTime);
         segmentToInterpolateFromStart.setLinearECMPVelocity();
         segmentToInterpolateFromEnd.getTimeInterval().setInterval(startTime, endTime);
         segmentToInterpolateFromEnd.setLinearECMPVelocity();


         FrameVector3DBasics velocity = new FrameVector3D();
         velocity.sub(endPoint, startPoint);
         velocity.scale(1.0 / duration);

         FramePoint3D positionFromStart = new FramePoint3D();
         FramePoint3D positionFromEnd = new FramePoint3D();

         positionFromStart.interpolate(startPoint, endPoint, alphaFromStart);
         positionFromEnd.interpolate(startPoint, endPoint, alphaFromEnd);

         helper.cubicInterpolateStartOfSegment(segmentToInterpolateFromStart, alphaFromStart);
         helper.cubicInterpolateEndOfSegment(segmentToInterpolateFromEnd, alphaFromEnd);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(positionFromStart, segmentToInterpolateFromStart.getECMPStartPosition(), positionEpsilon);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(velocity, segmentToInterpolateFromStart.getECMPStartVelocity(), velocityEpsilon);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(positionFromEnd, segmentToInterpolateFromEnd.getECMPEndPosition(), positionEpsilon);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(velocity, segmentToInterpolateFromEnd.getECMPEndVelocity(), velocityEpsilon);
      }
   }
}
