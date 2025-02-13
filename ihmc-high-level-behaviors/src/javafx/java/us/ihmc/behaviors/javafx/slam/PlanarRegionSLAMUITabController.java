package us.ihmc.behaviors.javafx.slam;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.control.*;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.SpinnerValueFactory.IntegerSpinnerValueFactory;
import javafx.stage.Window;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.behaviors.tools.perception.SimulatedREAModule;
import us.ihmc.behaviors.javafx.graphics.live.JavaFXLivePlanarRegionsGraphic;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAM;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMResult;
import us.ihmc.javafx.PrivateAnimationTimer;
import us.ihmc.javafx.RandomColorFunction;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ConcaveHullGraphicalMergerListener;
import us.ihmc.robotEnvironmentAwareness.tools.ConcaveHullMergerListener;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataExporter;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.io.WorkspacePathTools;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

public class PlanarRegionSLAMUITabController extends Group
{
   boolean showConcaveHullMergerListener = false;

   private PlanarRegionSLAMParameters planarRegionSLAMParameters;

   private static final String DATASET_1 = "20190710_174025_PlanarRegion";
   private static final String DATASET_2 = "20190710_174208_PlanarRegion";
//   private static final String DATASET_2 = "IntentionallyDrifted";
   private static final String DATASET_3 = "20190710_174422_PlanarRegion";

   @FXML private CheckBox acceptNewRegionListsCheckbox;
   @FXML private Button slamButton;
   @FXML private Button slamStepButton;
   @FXML private TextField slamStepStatus;

   @FXML private CheckBox fakeREAPublisherCheckbox;
   @FXML private RadioButton dataset1RadioButton;
   @FXML private RadioButton dataset2RadioButton;
   @FXML private RadioButton dataset3RadioButton;
   @FXML private RadioButton loadFromFileRadioButton;

   @FXML private Label parameterLabel1;
   @FXML private Label parameterLabel2;
   @FXML private Label parameterLabel3;
   @FXML private Label parameterLabel4;
   @FXML private Label parameterLabel5;

   @FXML private Spinner<Integer> parameterSpinner1;
   @FXML private Spinner<Double> parameterSpinner2;
   @FXML private Spinner<Double> parameterSpinner3;
   @FXML private Spinner<Double> parameterSpinner4;
   @FXML private Spinner<Double> parameterSpinner5;

   private List<RadioButton> datasetSelectionRadioButtons = new ArrayList<>();

   private PrivateAnimationTimer animationTimer = new PrivateAnimationTimer(this::fxUpdate);

   private Window window;
   private JavaFXLivePlanarRegionsGraphic livePlanarRegionsGraphic;
   private SimulatedREAModule simulatedREAModule;

   private PlanarRegionsList map = new PlanarRegionsList();
   private PlanarRegionsGraphic mapGraphic;

   private PlanarRegionSLAMGraphic visualizer;

   public void init(Window window, ROS2Node ros2Node)
   {
      this.window = window;

      planarRegionSLAMParameters = new PlanarRegionSLAMParameters();

      dataset1RadioButton.setText(DATASET_1);
      dataset2RadioButton.setText(DATASET_2);
      dataset3RadioButton.setText(DATASET_3);
      datasetSelectionRadioButtons.add(dataset1RadioButton);
      datasetSelectionRadioButtons.add(dataset2RadioButton);
      datasetSelectionRadioButtons.add(dataset3RadioButton);
      datasetSelectionRadioButtons.add(loadFromFileRadioButton);

      parameterLabel1.setText(PlanarRegionSLAMParameters.iterationsForMatching.getTitleCasedName());
      parameterLabel2.setText(PlanarRegionSLAMParameters.minimumNormalDotProduct.getTitleCasedName());
      parameterLabel3.setText(PlanarRegionSLAMParameters.dampedLeastSquaresLambda.getTitleCasedName());
      parameterLabel4.setText(PlanarRegionSLAMParameters.boundingBoxHeight.getTitleCasedName());
      parameterLabel5.setText(PlanarRegionSLAMParameters.minimumRegionOverlapDistance.getTitleCasedName());
      Platform.runLater(() ->
      {
      parameterSpinner1.setValueFactory(
            new IntegerSpinnerValueFactory(0, 100, planarRegionSLAMParameters.get(PlanarRegionSLAMParameters.iterationsForMatching), 1));
      parameterSpinner2.setValueFactory(
            new DoubleSpinnerValueFactory(0.5, 1.0, planarRegionSLAMParameters.get(PlanarRegionSLAMParameters.minimumNormalDotProduct), 0.01));
      parameterSpinner3.setValueFactory(
            new DoubleSpinnerValueFactory(0.0, 100.0, planarRegionSLAMParameters.get(PlanarRegionSLAMParameters.dampedLeastSquaresLambda), 0.5));
      parameterSpinner4.setValueFactory(
            new DoubleSpinnerValueFactory(0.0, 0.2, planarRegionSLAMParameters.get(PlanarRegionSLAMParameters.boundingBoxHeight), 0.005));
      parameterSpinner5.setValueFactory(
            new DoubleSpinnerValueFactory(-10.0, 10.0, planarRegionSLAMParameters.get(PlanarRegionSLAMParameters.minimumRegionOverlapDistance), 0.01));
      parameterSpinner1.getValueFactory().valueProperty().addListener(
            observable -> planarRegionSLAMParameters.set(PlanarRegionSLAMParameters.iterationsForMatching, parameterSpinner1.getValue()));
      parameterSpinner2.getValueFactory().valueProperty().addListener(
            observable -> planarRegionSLAMParameters.set(PlanarRegionSLAMParameters.minimumNormalDotProduct, parameterSpinner2.getValue()));
      parameterSpinner3.getValueFactory().valueProperty().addListener(
            observable -> planarRegionSLAMParameters.set(PlanarRegionSLAMParameters.dampedLeastSquaresLambda, parameterSpinner3.getValue()));
      parameterSpinner4.getValueFactory().valueProperty().addListener(
            observable -> planarRegionSLAMParameters.set(PlanarRegionSLAMParameters.boundingBoxHeight, parameterSpinner4.getValue()));
      parameterSpinner5.getValueFactory().valueProperty().addListener(
            observable -> planarRegionSLAMParameters.set(PlanarRegionSLAMParameters.minimumRegionOverlapDistance, parameterSpinner5.getValue()));
      });

      livePlanarRegionsGraphic = new JavaFXLivePlanarRegionsGraphic(ros2Node, false);
      livePlanarRegionsGraphic.setColorFunction(new RandomColorFunction()); // make incoming regions color always changing
      getChildren().add(livePlanarRegionsGraphic);

      mapGraphic = new PlanarRegionsGraphic(false);
      mapGraphic.generateMeshesAsync(map);
      getChildren().add(mapGraphic);

      visualizer = new PlanarRegionSLAMGraphic();
      visualizer.setStateListener(this::onVisualizerStateChange);

      simulatedREAModule = new SimulatedREAModule(loadDataSet(DATASET_1), PubSubImplementation.FAST_RTPS);

      animationTimer.start();
   }

   private void fxUpdate(long now)
   {
      mapGraphic.update();
   }

   private PlanarRegionsList loadDataSet(String dataSetName)
   {
      Path openRobotics = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software");
      Path path = openRobotics.resolve("robot-environment-awareness/Data/PlanarRegion/190710_SLAM_PlanarRegionFittingExamples/").resolve(dataSetName);
      return PlanarRegionFileTools.importPlanarRegionData(path.toFile());
   }

   private void setRadioButtonSelection(RadioButton selection)
   {
      datasetSelectionRadioButtons.forEach(it -> it.setSelected(false));
      selection.setSelected(true);
   }

   private void slam()
   {
      PlanarRegionsList newData = livePlanarRegionsGraphic.getLatestPlanarRegionsList();
      ConcaveHullMergerListener listener = showConcaveHullMergerListener ? new ConcaveHullGraphicalMergerListener() : null;
      PlanarRegionSLAMResult slamResult = PlanarRegionSLAM.slam(map, newData, planarRegionSLAMParameters, listener);
//      PlanarRegionSLAMResult slamResult = PlanarRegionSLAM.intentionallyDrift(livePlanarRegionsGraphic.getLatestPlanarRegionsList());

      RigidBodyTransform transformFromIncomingToMap = slamResult.getTransformFromIncomingToMap();
      LogTools.info("\nSlam result: transformFromIncomingToMap = \n" + transformFromIncomingToMap);

      map = slamResult.getMergedMap();
      mapGraphic.generateMeshesAsync(map);

      slamStepButton.setDisable(false);  // map must exist before step enabled
   }

   private void onVisualizerStateChange(PlanarRegionSLAMGraphic.SLAMVisualizationState state)
   {
      slamStepStatus.setText(state.name());

      if (state == PlanarRegionSLAMGraphic.SLAMVisualizationState.Hidden)
      {
         slamButton.setDisable(false);
         livePlanarRegionsGraphic.setAcceptNewRegions(acceptNewRegionListsCheckbox.isSelected());
         acceptNewRegionListsCheckbox.setDisable(false);

         getChildren().add(mapGraphic);
         getChildren().add(livePlanarRegionsGraphic);
         getChildren().remove(visualizer);
      }
   }

   @FXML private void acceptNewRegionListsCheckbox()
   {
      livePlanarRegionsGraphic.setAcceptNewRegions(acceptNewRegionListsCheckbox.isSelected());
   }

   @FXML private void slamButton()
   {
      ThreadTools.startAThread(this::slam, "SLAM");
   }

   @FXML private void slamStepButton()
   {
      LogTools.info("slamStepButton() visualizer: {}", visualizer.getState().name());
      if (visualizer.getState() == PlanarRegionSLAMGraphic.SLAMVisualizationState.Hidden)
      {
         slamButton.setDisable(true);
         livePlanarRegionsGraphic.setAcceptNewRegions(false);
         acceptNewRegionListsCheckbox.setDisable(true);

         visualizer.copyDataIn(map, livePlanarRegionsGraphic.getLatestPlanarRegionsList());

         getChildren().remove(mapGraphic);
         getChildren().remove(livePlanarRegionsGraphic);
         getChildren().add(visualizer);
      }

      visualizer.step();
   }

   @FXML private void exportMapButton()
   {
      PlanarRegionDataExporter.exportUsingFileChooser(window, map);
   }

   @FXML private void clearMapButton()
   {
      map.clear();
      mapGraphic.generateMeshesAsync(map);
   }

   @FXML private void clearIncomingButton()
   {
      livePlanarRegionsGraphic.clear();
   }

   @FXML private void exportIncomingButton()
   {
      PlanarRegionDataExporter.exportUsingFileChooser(window, livePlanarRegionsGraphic.getLatestPlanarRegionsList());
   }

   @FXML private void fakeREAPublisherCheckbox()
   {
      if (fakeREAPublisherCheckbox.isSelected())
      {
         simulatedREAModule.start();
      }
      else
      {
         simulatedREAModule.stop();
      }
   }

   @FXML private void dataset1RadioButton()
   {
      setRadioButtonSelection(dataset1RadioButton);
      simulatedREAModule.setMap(loadDataSet(DATASET_1));
   }

   @FXML private void dataset2RadioButton()
   {
      setRadioButtonSelection(dataset2RadioButton);
      simulatedREAModule.setMap(loadDataSet(DATASET_2));
   }

   @FXML private void dataset3RadioButton()
   {
      setRadioButtonSelection(dataset3RadioButton);
      simulatedREAModule.setMap(loadDataSet(DATASET_3));
   }

   @FXML private void loadFromFileRadioButton()
   {
      setRadioButtonSelection(loadFromFileRadioButton);
      simulatedREAModule.setMap(PlanarRegionDataImporter.importUsingFileChooser(window));
   }

   @FXML private void saveParametersButton()
   {
      planarRegionSLAMParameters.save();
   }
}
