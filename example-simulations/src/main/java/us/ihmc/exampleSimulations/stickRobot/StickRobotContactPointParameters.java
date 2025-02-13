package us.ihmc.exampleSimulations.stickRobot;

import static us.ihmc.exampleSimulations.stickRobot.StickRobotPhysicalProperties.footLength;
import static us.ihmc.exampleSimulations.stickRobot.StickRobotPhysicalProperties.footWidth;
import static us.ihmc.exampleSimulations.stickRobot.StickRobotPhysicalProperties.soleToAnkleFrameTransforms;

import java.util.ArrayList;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.modelFileLoaders.ModelFileLoaderConversionsHelper;
import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.SDFJointHolder;
import us.ihmc.modelFileLoaders.SdfLoader.SDFLinkHolder;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.Collision;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFGeometry.Sphere;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.wholeBodyController.FootContactPoints;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class StickRobotContactPointParameters extends RobotContactPointParameters<RobotSide>
{
   private final SideDependentList<ArrayList<Point2D>> footGroundContactPoints = new SideDependentList<>();

   private final HumanoidJointNameMap jointMap;

   public StickRobotContactPointParameters(HumanoidJointNameMap jointMap, FootContactPoints<RobotSide> footContactPoints)
   {
      super(jointMap, footWidth, footLength, soleToAnkleFrameTransforms);
      this.jointMap = jointMap;

      if (footContactPoints == null)
         createDefaultFootContactPoints();
      else
         createFootContactPoints(footContactPoints);
   }

   private void checkJointChildren(SDFJointHolder joint)
   {
      SDFLinkHolder link = joint.getChildLinkHolder();
      for (Collision collision : link.getCollisions())
      {
         String name = collision.getName();
         Sphere sphere = collision.getGeometry().getSphere();
         if (name.contains("_heel") || name.contains("_toe") || name.contains("sim_contact") || (sphere != null && Double.parseDouble(sphere.getRadius()) == 0.0))
         {
            System.out.println("Simulation contact '" + name + "'");
            Vector3D gcOffset = new Vector3D();

            gcOffset.set(ModelFileLoaderConversionsHelper.poseToTransform(collision.getPose()).getTranslation());
            link.getTransformFromModelReferenceFrame().transform(gcOffset);
            addSimulationContactPoint(joint.getName(), gcOffset);
         }

         if (name.contains("ctrl_contact"))
         {
            System.out.println("Controller contact '" + name + "'");
            Vector3D gcOffset = new Vector3D();

            gcOffset.set(ModelFileLoaderConversionsHelper.poseToTransform(collision.getPose()).getTranslation());
            link.getTransformFromModelReferenceFrame().transform(gcOffset);
            boolean assigned = false;

            for (RobotSide robotSide : RobotSide.values)
            {
               if (joint.getName().equals(jointMap.getJointBeforeFootName(robotSide)))
               {
                  footGroundContactPoints.get(robotSide).add(projectOnPlane(StickRobotPhysicalProperties.getSoleToAnkleFrameTransform(robotSide), gcOffset));
                  assigned = true;
                  break;
               }
               else if (joint.getName().equals(jointMap.getJointBeforeHandName(robotSide)))
               {
                  System.err.println("Hand contacts are not supported (" + name + ")");
                  assigned = true;
                  break;
               }
               else if (joint.getName().equals(jointMap.getChestName()))
               {
                  System.err.println("Chest contacts are not supported (" + name + ")");
                  assigned = true;
                  break;
               }
               else if (joint.getName().equals(jointMap.getPelvisName()))
               {
                  System.err.println("Pelvis contacts are not supported (" + name + ")");
                  // Pelvis back has to be disnguished here
                  assigned = true;
                  break;
               }
               else if (joint.getName().equals(jointMap.getNameOfJointBeforeThighs().get(robotSide)))
               {
                  System.err.println("Thigh contacts are not supported (" + name + ")");
                  assigned = true;
                  break;
               }
            }
            if (!assigned)
            {
               System.err.println("Contacts with '" + joint.getName() + "' are not supported (" + name + ")");
            }
         }
      }

      for (SDFJointHolder child : link.getChildren())
      {
         checkJointChildren(child);
      }
   }

   private Point2D projectOnPlane(RigidBodyTransform plane, Vector3D point)
   {
      RigidBodyTransform planeInv = new RigidBodyTransform(plane);
      planeInv.invert();
      planeInv.transform(point);
      return new Point2D(point.getX(), point.getY());
   }

   public void setupContactPointsFromRobotModel(GeneralizedSDFRobotModel sdf, boolean removeExistingContacts)
   {
      if (removeExistingContacts)
         clearSimulationContactPoints();

      for (SDFLinkHolder link : sdf.getRootLinks())
      {
         for (SDFJointHolder joint : link.getChildren())
         {
            checkJointChildren(joint);
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         if (!footGroundContactPoints.get(robotSide).isEmpty())
         {
            clearControllerFootContactPoints();
            setControllerFootContactPoint(robotSide, footGroundContactPoints.get(robotSide));
         }
      }
   }
}
