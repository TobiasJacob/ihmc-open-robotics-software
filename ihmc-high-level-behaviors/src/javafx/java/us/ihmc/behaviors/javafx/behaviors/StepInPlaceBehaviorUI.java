package us.ihmc.behaviors.javafx.behaviors;

import javafx.fxml.FXML;
import javafx.scene.SubScene;
import javafx.scene.layout.Pane;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.javafx.JavaFXBehaviorUIDefinition;
import us.ihmc.behaviors.javafx.JavaFXBehaviorUIInterface;
import us.ihmc.behaviors.StepInPlaceBehavior;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.ROS2NodeInterface;

public class StepInPlaceBehaviorUI extends JavaFXBehaviorUIInterface
{
   public static final JavaFXBehaviorUIDefinition DEFINITION = new JavaFXBehaviorUIDefinition(StepInPlaceBehavior.DEFINITION, StepInPlaceBehaviorUI::new);

   public StepInPlaceBehaviorUI(SubScene sceneNode, Pane visualizationPane, ROS2NodeInterface ros2Node, Messager behaviorMessager, DRCRobotModel robotModel)
   {
      super(sceneNode, visualizationPane, ros2Node, behaviorMessager, robotModel);
   }

   @Override
   public void setEnabled(boolean enabled)
   {

   }

   @FXML public void startStepping()
   {
      getBehaviorMessager().submitMessage(StepInPlaceBehavior.API.Stepping, true);
   }

   @FXML public void pauseStepping()
   {
      getBehaviorMessager().submitMessage(StepInPlaceBehavior.API.Stepping, false);
   }

   @Override
   public void destroy()
   {

   }
}
