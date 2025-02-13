package us.ihmc.behaviors.javafx.graphics;

import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.javafx.JavaFXGraphicTools;

public class OrientationGraphic
{
   private final MeshView arrow;
   private final FramePose3D pose = new FramePose3D();

   public OrientationGraphic()
   {
      this(Color.GREEN, 0.25);
   }

   public OrientationGraphic(Color color, double cylinderLength)
   {
      double radius = cylinderLength / 20.0;

      TextureColorPalette1D colorPalette = new TextureColorPalette1D();
      colorPalette.setHueBased(1.0, 1.0);
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

      double coneHeight = 0.10 * cylinderLength;
      double coneRadius = 1.5 * radius;

      meshBuilder.addCylinder(cylinderLength, radius, new Point3D(), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), color);
      meshBuilder.addCone(coneHeight, coneRadius, new Point3D(cylinderLength, 0.0, 0.0), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), color);

      arrow = new MeshView(meshBuilder.generateMesh());
      arrow.setMaterial(meshBuilder.generateMaterial());
   }

   public FramePose3DBasics getPose()
   {
      return pose;
   }

   public void setPose(Pose3DReadOnly pose)
   {
      this.pose.set(pose);
      update();
   }

   public void setPosition(Point3DReadOnly position)
   {
      this.pose.getPosition().set(position);
      update();
   }

   public void setOrientation(Orientation3DReadOnly orientation)
   {
      this.pose.getOrientation().set(orientation);
      update();
   }

   public void update()
   {
      JavaFXGraphicTools.setNodeTransformFromPose(arrow, pose);
   }

   public void setVisible(boolean visible)
   {
      arrow.setVisible(visible);
   }

   public Node getNode()
   {
      return arrow;
   }

   public double getYaw()
   {
      return pose.getYaw();
   }
}
