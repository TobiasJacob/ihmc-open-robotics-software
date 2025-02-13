package us.ihmc.robotics.geometry.algorithms;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameSphere3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;

public class SphereWithConvexPolygonIntersector
{
   private final FramePoint3D closestPointOnPolygon;
   private final FramePoint2D closestPointOnPolygon2d;

   public SphereWithConvexPolygonIntersector()
   {
      closestPointOnPolygon = new FramePoint3D();
      closestPointOnPolygon2d = new FramePoint2D();
   }

   /**
    * All math in polygon frame.
    */
   public boolean checkIfIntersectionExists(FrameSphere3D sphere, FrameConvexPolygon2D polygon)
   {
      ReferenceFrame originalSphereFrame = sphere.getReferenceFrame();
      sphere.changeFrame(polygon.getReferenceFrame());
      
      closestPointOnPolygon.setIncludingFrame(sphere.getPosition());
      closestPointOnPolygon2d.setIncludingFrame(closestPointOnPolygon);
      
      Point2DBasics pointUnsafe = closestPointOnPolygon2d;
      polygon.orthogonalProjection(pointUnsafe);
      closestPointOnPolygon2d.set(pointUnsafe.getX(), pointUnsafe.getY());
      
      closestPointOnPolygon.set(closestPointOnPolygon2d, 0.0);
      
      boolean isInsideOrOnSurface = sphere.isPointInside(closestPointOnPolygon);
      
      closestPointOnPolygon.changeFrame(ReferenceFrame.getWorldFrame());
      sphere.changeFrame(originalSphereFrame);
      
      return isInsideOrOnSurface;
   }

   public FramePoint3D getClosestPointOnPolygon()
   {
      return closestPointOnPolygon;
   }
}
