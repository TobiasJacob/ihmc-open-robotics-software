package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import static controller_msgs.msg.dds.KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_INITIALIZE_FAILURE_MISSING_RCD;
import static controller_msgs.msg.dds.KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_INITIALIZE_SUCCESSFUL;
import static controller_msgs.msg.dds.KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_RUNNING;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import controller_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage;
import controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataHolderReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerTemplate;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsOptimizationSettingsCommand.JointVelocityLimitMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.LinearMomentumConvexConstraint2DCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBPoint3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBQuaternion3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.Type;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.ListWrappingIndexTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.lists.SupplierBuilder;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.collision.EuclidFrameShape3DCollisionResult;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxContactStateCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxInputCollectionCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxOneDoFJointCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxPrivilegedConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPIDGains;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.CollisionResult;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.time.ThreadTimer;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * {@code KinematicsToolboxController} is used as a whole-body inverse kinematics solver.
 * <p>
 * The interaction with this solver is achieved over the network using message that define the API,
 * see {@link KinematicsToolboxModule}.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class KinematicsToolboxController extends ToolboxController
{
   private static final double GLOBAL_PROPORTIONAL_GAIN = 1200.0;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double DEFAULT_PRIVILEGED_CONFIGURATION_WEIGHT = 0.025;
   private static final double DEFAULT_PRIVILEGED_CONFIGURATION_GAIN = 50.0;

   /**
    * Indicates the duration of a control tick. It should match the thread period in
    * {@link ToolboxModule}.
    */
   protected final double updateDT;

   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   /** Reference to the desired robot's root body. */
   protected final RigidBodyBasics rootBody;
   /** Reference to the desired robot's floating joint. */
   protected final FloatingJointBasics rootJoint;
   private final double totalRobotMass;
   /**
    * Array containing all the one degree-of-freedom joints of the desired robot except for the finger
    * joints that are not handled by this solver.
    */
   private final OneDoFJointBasics[] oneDoFJoints;
   private final Collection<? extends RigidBodyBasics> controllableRigidBodies;

   /**
    * Reference frame centered at the robot's center of mass. It is used to hold the initial center of
    * mass position when requested.
    */
   protected final ReferenceFrame centerOfMassFrame;

   /** The same set of gains is used for controlling any part of the desired robot body. */
   private final YoPIDSE3Gains spatialGains = new DefaultYoPIDSE3Gains("GenericSpatialGains", GainCoupling.XYZ, false, registry);
   /** The same set of gains is used for controlling any joint of the desired robot body. */
   private final YoPIDGains jointGains = new YoPIDGains("GenericJointGains", registry);
   /**
    * Default settings for the solver. The joint velocity/acceleration weights can be modified at
    * runtime using {@link KinematicsToolboxConfigurationMessage}.
    */
   private final KinematicsToolboxOptimizationSettings optimizationSettings = new KinematicsToolboxOptimizationSettings();
   /** Command carrying the current optimization settings. */
   private final InverseKinematicsOptimizationSettingsCommand activeOptimizationSettings = new InverseKinematicsOptimizationSettingsCommand();
   /**
    * The controller core command is the single object used to pass the desired inputs to the
    * controller core.
    */
   private final ControllerCoreCommandBuffer controllerCoreCommand = new ControllerCoreCommandBuffer();
   /**
    * This is where the magic is happening. The controller is responsible for performing feedback
    * control to reduce the difference between the desireds sent to the
    * {@code KinematicsToolboxController} and the actual robot pose. It is also responsible for
    * gathering the entire set of desired inputs and formulate the adequate optimization problem to be
    * solved for every control tick. The output of the controller core provides every tick a new robot
    * joint configurations and velocities that are one step closer to the desireds. The output is used
    * to update the state of the {@link #desiredFullRobotModel} such that it progresses towards the
    * desired user inputs over time.
    */
   private final WholeBodyControllerCore controllerCore;
   /**
    * This holds onto the data from the feedback controllers running inside the controller core.
    * {@link #feedbackControllerDataHolder} is used here to compute the solution quality every tick
    * from the tracking error for each end-effector being controlled.
    */
   private final FeedbackControllerDataHolderReadOnly feedbackControllerDataHolder;

   /**
    * This is the output of the {@code KinematicsToolboxController}. It is filled with the robot
    * configuration obtained from {@link #desiredFullRobotModel} and also with the solution quality
    * which can be used to quickly see if the solution is viable. It is sent back to the caller only.
    */
   private final KinematicsToolboxOutputStatus inverseKinematicsSolution;
   /** Variable to keep track of when the last solution was published. */
   private final YoDouble timeLastSolutionPublished = new YoDouble("timeLastSolutionPublished", registry);
   /** Specifies time interval for publishing the solution. */
   private final YoDouble publishSolutionPeriod = new YoDouble("publishSolutionPeriod", registry);
   /**
    * This is the current estimate of the solution quality that is calculated based on the tracking
    * error for the end-effectors (center of mass included) being actively controlled.
    */
   private final YoDouble solutionQuality = new YoDouble("solutionQuality", registry);
   private final KinematicsSolutionQualityCalculator solutionQualityCalculator = new KinematicsSolutionQualityCalculator();
   private final FeedbackControlCommandList allFeedbackControlCommands = new FeedbackControlCommandList();

   /**
    * Weight indicating the priority for getting closer to the current privileged configuration. The
    * current privileged configuration can be changed at any time by sending a
    * {@link HumanoidKinematicsToolboxConfigurationMessage}.
    */
   private final YoDouble privilegedWeight = new YoDouble("privilegedWeight", registry);
   /**
    * To make the robot get closer to the privileged configuration, a feedback control is used to
    * compute for each joint a privileged velocity based on the difference between the privileged angle
    * and the current angle. These privileged joint velocities are then used to complete the
    * optimization problem in such way that they don't interfere with the user commands.
    */
   private final YoDouble privilegedConfigurationGain = new YoDouble("privilegedConfigurationGain", registry);
   /**
    * Cap used to limit the magnitude of the privileged joint velocities computed in the controller
    * core. Should probably remain equal to {@link Double#POSITIVE_INFINITY} so the solution converges
    * quicker.
    */
   private final YoDouble privilegedMaxVelocity = new YoDouble("privilegedMaxVelocity", registry);
   /**
    * Defines a robot configuration the this IK start from and also defines the privileged joint
    * configuration.
    */
   protected TObjectDoubleHashMap<OneDoFJointBasics> initialRobotConfigurationMap = null;
   private boolean submitPrivilegedConfigurationCommand = true;
   /**
    * This reference to {@link PrivilegedConfigurationCommand} is used internally only to figure out if
    * the current privileged configuration used in the controller core is to be updated or not. It is
    * usually updated once right after the initialization phase.
    */
   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   /**
    * The {@link #commandInputManager} is used as a 'thread-barrier'. When receiving a new user input,
    * this manager automatically copies the data in the corresponding command that can then be used
    * here safely.
    */
   protected final CommandInputManager commandInputManager;
   private final List<RigidBodyBasics> rigidBodiesWithVisualization = new ArrayList<>();
   /**
    * Visualization of the desired end-effector poses seen as coordinate systems in the
    * {@code SCSVisualizer}. They are only visible when the end-effector is being actively controlled.
    */
   private final Map<RigidBodyBasics, YoGraphicCoordinateSystem> desiredCoodinateSystems = new HashMap<>();
   /**
    * Visualization of the current end-effector poses seen as coordinate systems in the
    * {@code SCSVisualizer}. They are only visible when the end-effector is being actively controlled.
    */
   private final Map<RigidBodyBasics, YoGraphicCoordinateSystem> currentCoodinateSystems = new HashMap<>();

   /**
    * Reference to the most recent robot configuration received from the controller. It is used for
    * initializing the {@link #desiredFullRobotModel} before starting the optimization process.
    */
   private final ConcurrentCopier<RobotConfigurationData> concurrentRobotConfigurationDataCopier = new ConcurrentCopier<>(RobotConfigurationData::new);
   protected final RobotConfigurationData robotConfigurationDataInternal = new RobotConfigurationData();

   /** Command buffer used to keep track of the current commands submitted by the user. */
   private final FeedbackControlCommandBuffer userFBCommands = new FeedbackControlCommandBuffer();
   /** Command buffer used to keep track of the commands previously submitted by the user. */
   private final FeedbackControlCommandBuffer previousUserFBCommands = new FeedbackControlCommandBuffer();

   private final YoBoolean isUserProvidingSupportPolygon = new YoBoolean("isUserProvidingSupportPolygon", registry);

   /** Intermediate variable for garbage-free operation. */
   private final List<FramePoint3DReadOnly> contactPointLocations = new ArrayList<>();
   /** The active support polygon updated from the most recent robot configuration. */
   private final ConvexPolygon2D supportPolygon = new ConvexPolygon2D();
   /**
    * The active support polygon shrunk by the distance {@code centerOfMassSafeMargin}. This represents
    * the convex horizontal region that the center of mass is constrained to.
    */
   private final RecyclingArrayList<Point2D> shrunkSupportPolygonVertices = new RecyclingArrayList<>(Point2D.class);
   /** Helper used for shrink the support polygon. */
   private final ConvexPolygonScaler convexPolygonScaler = new ConvexPolygonScaler();
   private final FrameConvexPolygon2D newSupportPolygon = new FrameConvexPolygon2D();
   private final ConvexPolygon2D shrunkConvexPolygon = new ConvexPolygon2D();
   private final FramePoint2D centerOfMass = new FramePoint2D();
   /** Distance to shrink the support polygon for safety purpose. */
   private final YoDouble centerOfMassSafeMargin = new YoDouble("centerOfMassSafeMargin",
                                                                "Describes the minimum distance away from the support polygon's edges.",
                                                                registry);
   /** The total mass of the robot. */
   private final double robotMass;

   /**
    * Indicates whether the projection of the center of mass is to be contained inside the support
    * polygon.It is {@code true} by default but can be disabled using the message
    * {@link KinematicsToolboxConfigurationMessage}.
    */
   protected final YoBoolean enableSupportPolygonConstraint = new YoBoolean("enableSupportPolygonConstraint", registry);

   /**
    * This is mostly for visualization to be able to keep track of the number of commands that the user
    * submitted.
    */
   private final YoInteger numberOfActiveCommands = new YoInteger("numberOfActiveCommands", registry);
   /**
    * When {@code true}, a user command will persist over time until the method {@link #initialize()}
    * is called.
    */
   private final YoBoolean preserveUserCommandHistory = new YoBoolean("preserveUserCommandHistory", registry);

   /** Represents the collision model of the robot. */
   private final List<Collidable> robotCollidables = new ArrayList<>();
   /**
    * User parameter updated via {@link KinematicsToolboxConfigurationMessage}. Collision is only
    * handled when this is set to {@code true}.
    */
   private final YoBoolean enableCollisionAvoidance = new YoBoolean("enableCollisionAvoidance", registry);
   private final RecyclingArrayList<CollisionResult> collisionResults = new RecyclingArrayList<>(CollisionResult::new);
   private final RecyclingArrayList<KinematicsCollisionFrame> collisionFrames = new RecyclingArrayList<>(SupplierBuilder.indexedSupplier(collisionIndex ->
   {
      return new KinematicsCollisionFrame("collisionFrame" + collisionIndex, worldFrame);
   }));
   /**
    * Threshold for activating collision response, i.e. when 2 collidables are within a distance that
    * is less than this value, only then the solver handles it. This is for reducing computational
    * burden.
    */
   private final YoDouble collisionActivationDistanceThreshold = new YoDouble("collisionActivationDistanceThreshold", registry);
   private final YoDouble maxCollisionResolutionVelocity = new YoDouble("maxCollisionResolutionVelocity", registry);
   /** Sets the maximum number of collisions to create YoVariables for. */
   private final int numberOfCollisionsToVisualize = 20;
   /** Debug variable. */
   private final YoDouble[] yoCollisionDistances = new YoDouble[numberOfCollisionsToVisualize];
   /** Debug variable. */
   private final YoFramePoint3D[] yoCollisionPointAs = new YoFramePoint3D[numberOfCollisionsToVisualize];
   /** Debug variable. */
   private final YoFramePoint3D[] yoCollisionPointBs = new YoFramePoint3D[numberOfCollisionsToVisualize];
   /** Debug variable. */
   private final YoFramePose3D[] yoCollisionFramePoses = new YoFramePose3D[numberOfCollisionsToVisualize];
   /** Timer to debug computational load. */
   private final ThreadTimer threadTimer;

   /**
    * When {@code true}, the solver will add an objective to minimize the overall angular momentum
    * generated. This is not recommended when using this toolbox as an IK solver as it'll increase the
    * number of iterations before converging.
    */
   private final YoBoolean minimizeAngularMomentum = new YoBoolean("minimizeAngularMomentum", registry);
   /**
    * The weight to be used for minimizing the angular momentum, around 0.1 seems good for a robot that
    * is about 130kg.
    */
   private final YoDouble angularMomentumWeight = new YoDouble("angularMomentumWeight", registry);
   private final MomentumCommand angularMomentumCommand = new MomentumCommand();

   /**
    * @param commandInputManager     the message/command barrier used by this controller. Submit
    *                                messages or commands to be processed to the
    *                                {@code commandInputManager} from outside the controller.
    * @param statusOutputManager     the output interface used by this controller.
    * @param rootJoint               the underactuated floating root joint of the multi-body system.
    *                                Can be {@code null} in the case all the joints are actuated.
    * @param oneDoFJoints            the actuated joints of the system. The inverse kinematics will
    *                                only use these joints during the optimization.
    * @param controllableRigidBodies the sublist of rigid-bodies that can be controlled by the user.
    *                                Can be {@code null} in the case all rigid-body should be
    *                                controllable.
    * @param updateDT                the period of one optimization tick.
    * @param yoGraphicsListRegistry  registry to register visualization to.
    * @param parentRegistry          registry to attach {@code YoVariable}s to.
    */
   public KinematicsToolboxController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager, FloatingJointBasics rootJoint,
                                      OneDoFJointBasics[] oneDoFJoints, Collection<? extends RigidBodyBasics> controllableRigidBodies, double updateDT,
                                      YoGraphicsListRegistry yoGraphicsListRegistry, YoRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);
      this.commandInputManager = commandInputManager;
      this.rootJoint = rootJoint;
      this.oneDoFJoints = oneDoFJoints;
      this.controllableRigidBodies = controllableRigidBodies;
      this.updateDT = updateDT;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      // This will find the root body without using rootJoint so it can be null.
      rootBody = MultiBodySystemTools.getRootBody(oneDoFJoints[0].getPredecessor());
      totalRobotMass = TotalMassCalculator.computeSubTreeMass(rootBody);

      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", worldFrame, rootBody);

      controllerCoreCommand.setControllerCoreMode(WholeBodyControllerCoreMode.INVERSE_KINEMATICS);
      controllerCore = createControllerCore(controllableRigidBodies);
      feedbackControllerDataHolder = controllerCore.getWholeBodyFeedbackControllerDataHolder();

      inverseKinematicsSolution = MessageTools.createKinematicsToolboxOutputStatus(oneDoFJoints);
      inverseKinematicsSolution.setDestination(-1);

      robotMass = TotalMassCalculator.computeSubTreeMass(rootBody);
      centerOfMassSafeMargin.set(0.04); // Same as the walking controller.

      spatialGains.setPositionProportionalGains(GLOBAL_PROPORTIONAL_GAIN); // Gains used for everything. It is as high as possible to reduce the convergence time.
      spatialGains.setPositionMaxFeedbackAndFeedbackRate(1500.0, Double.POSITIVE_INFINITY);
      spatialGains.setOrientationProportionalGains(GLOBAL_PROPORTIONAL_GAIN); // Gains used for everything. It is as high as possible to reduce the convergence time.
      spatialGains.setOrientationMaxFeedbackAndFeedbackRate(1500.0, Double.POSITIVE_INFINITY);

      jointGains.setKp(GLOBAL_PROPORTIONAL_GAIN); // Gains used for everything. It is as high as possible to reduce the convergence time.
      jointGains.setMaximumFeedbackAndMaximumFeedbackRate(1500.0, Double.POSITIVE_INFINITY);

      privilegedWeight.set(DEFAULT_PRIVILEGED_CONFIGURATION_WEIGHT);
      privilegedConfigurationGain.set(DEFAULT_PRIVILEGED_CONFIGURATION_GAIN);
      privilegedMaxVelocity.set(Double.POSITIVE_INFINITY);

      publishSolutionPeriod.set(0.01);
      preserveUserCommandHistory.set(true);

      threadTimer = new ThreadTimer("timer", updateDT, registry);

      minimizeAngularMomentum.set(false);
      angularMomentumWeight.set(0.125);
      angularMomentumCommand.setSelectionMatrixForAngularControl();

      enableCollisionAvoidance.set(true);
      collisionActivationDistanceThreshold.set(0.10);
      maxCollisionResolutionVelocity.set(0.10);
      setupCollisionVisualization();
   }

   /**
    * Creates the debug variables and graphics for the collisions.
    */
   private void setupCollisionVisualization()
   {
      Random random = new Random();

      for (int i = 0; i < numberOfCollisionsToVisualize; i++)
      {
         YoDouble collisionDistance = new YoDouble("collision_" + i + "_distance", registry);
         YoFramePoint3D collisionPointA = new YoFramePoint3D("collision_" + i + "_pointA" + i, worldFrame, registry);
         YoFramePoint3D collisionPointB = new YoFramePoint3D("collision_" + i + "_pointB" + i, worldFrame, registry);
         YoFramePose3D collisionFramePose = new YoFramePose3D("collision_" + i + "_frame", worldFrame, registry);

         if (yoGraphicsListRegistry != null)
         {
            AppearanceDefinition appearance = new YoAppearanceRGBColor(new Color(random.nextInt()), 0.7);
            yoGraphicsListRegistry.registerYoGraphic("Collisions", new YoGraphicPosition("collision_" + i + "_pointA", collisionPointA, 0.01, appearance));
            yoGraphicsListRegistry.registerYoGraphic("Collisions", new YoGraphicPosition("collision_" + i + "_pointB", collisionPointB, 0.01, appearance));
            yoGraphicsListRegistry.registerYoGraphic("Collisions",
                                                     new YoGraphicCoordinateSystem("collision_" + i + "_frame", collisionFramePose, 0.1, appearance));
         }

         yoCollisionDistances[i] = collisionDistance;
         yoCollisionPointAs[i] = collisionPointA;
         yoCollisionPointBs[i] = collisionPointB;
         yoCollisionFramePoses[i] = collisionFramePose;
      }
   }

   /**
    * Sets up the robot configuration this IK should start from when initializing.
    * 
    * @param initialRobotConfigurationMap the map from joint to initial joint position.
    */
   public void setInitialRobotConfiguration(Map<OneDoFJointBasics, Double> initialRobotConfigurationMap)
   {
      if (initialRobotConfigurationMap == null)
      {
         this.initialRobotConfigurationMap = null;
         return;
      }

      this.initialRobotConfigurationMap = new TObjectDoubleHashMap<>();
      initialRobotConfigurationMap.entrySet().forEach(entry -> this.initialRobotConfigurationMap.put(entry.getKey(), entry.getValue()));
   }

   /**
    * Sets up the robot configuration this IK should start from when initializing.
    * 
    * @param jointNameToInitialJointPosition the map from joint name to initial joint position.
    */
   public void setInitialRobotConfigurationNamedMap(Map<String, Double> jointNameToInitialJointPosition)
   {
      if (jointNameToInitialJointPosition == null)
      {
         this.initialRobotConfigurationMap = null;
         return;
      }
      this.initialRobotConfigurationMap = new TObjectDoubleHashMap<>();

      for (OneDoFJointBasics joint : oneDoFJoints)
      {
         Double q_priv = jointNameToInitialJointPosition.get(joint.getName());
         if (q_priv != null)
            initialRobotConfigurationMap.put(joint, q_priv);
      }
   }

   /**
    * Registers a new collidable to be used with this solver for preventing collisions.
    * 
    * @param collidable the new collidable to consider.
    */
   public void registerCollidable(Collidable collidable)
   {
      robotCollidables.add(collidable);
   }

   /**
    * Registers new collidables to be used with this solver for preventing collisions.
    * 
    * @param collidables the new collidables to consider.
    */
   public void registerCollidables(Collidable... collidables)
   {
      for (Collidable collidable : collidables)
         robotCollidables.add(collidable);
   }

   /**
    * Registers new collidables to be used with this solver for preventing collisions.
    * 
    * @param collidables the new collidables to consider.
    */
   public void registerCollidables(Iterable<? extends Collidable> collidables)
   {
      for (Collidable collidable : collidables)
         robotCollidables.add(collidable);
   }

   /**
    * This is where the end-effectors needing a visualization are registered, if you need more, add it
    * there.
    * 
    * @param rigidBodies all the rigid bodies for which the desired and actual pose will be displayed
    *                    using graphical coordinate systems.
    */
   public void setupVisualization(RigidBodyBasics... rigidBodies)
   {
      AppearanceDefinition desiredAppearance = YoAppearance.Red();
      AppearanceDefinition currentAppearance = YoAppearance.Blue();

      for (RigidBodyBasics rigidBody : rigidBodies)
      {
         YoGraphicCoordinateSystem desiredCoodinateSystem = createCoodinateSystem(rigidBody, Type.DESIRED, desiredAppearance);
         YoGraphicCoordinateSystem currentCoodinateSystem = createCoodinateSystem(rigidBody, Type.CURRENT, currentAppearance);

         rigidBodiesWithVisualization.add(rigidBody);
         desiredCoodinateSystems.put(rigidBody, desiredCoodinateSystem);
         currentCoodinateSystems.put(rigidBody, currentCoodinateSystem);

         yoGraphicsListRegistry.registerYoGraphic("CoordinateSystems", desiredCoodinateSystem);
         yoGraphicsListRegistry.registerYoGraphic("CoordinateSystems", currentCoodinateSystem);
      }
   }

   /**
    * Convenience method that should only be used for setting up the visualization.
    * <p>
    * Simply creates a graphic coordinate system visualizable in {@code SCSVisualizer}.
    * </p>
    *
    * @param endEffector          used to create a name prefix required for creating a
    *                             {@link YoGraphicCoordinateSystem}.
    * @param type                 used to create a name prefix required for creating a
    *                             {@link YoGraphicCoordinateSystem}.
    * @param appearanceDefinition the appearance of the coordinate system's arrows.
    * @return the graphic with a good name for the given end-effector.
    */
   private YoGraphicCoordinateSystem createCoodinateSystem(RigidBodyBasics endEffector, Type type, AppearanceDefinition appearanceDefinition)
   {
      String namePrefix = endEffector.getName() + type.getName();
      return new YoGraphicCoordinateSystem(namePrefix, "", registry, false, 0.2, appearanceDefinition);
   }

   /**
    * Creating the controller core which is the main piece of this solver.
    * 
    * @param controllableRigidBodies
    * @return the controller core that will run for the desired robot {@link #desiredFullRobotModel}.
    */
   private WholeBodyControllerCore createControllerCore(Collection<? extends RigidBodyBasics> controllableRigidBodies)
   {
      JointBasics[] controlledJoints;
      if (rootJoint != null)
      {
         controlledJoints = new JointBasics[oneDoFJoints.length + 1];
         controlledJoints[0] = rootJoint;
         System.arraycopy(oneDoFJoints, 0, controlledJoints, 1, oneDoFJoints.length);
      }
      else
      {
         controlledJoints = oneDoFJoints;
      }
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(updateDT,
                                                                            0.0,
                                                                            rootJoint,
                                                                            controlledJoints,
                                                                            centerOfMassFrame,
                                                                            optimizationSettings,
                                                                            null,
                                                                            registry);
      toolbox.setJointPrivilegedConfigurationParameters(new JointPrivilegedConfigurationParameters());
      toolbox.setupForInverseKinematicsSolver();
      FeedbackControllerTemplate controllerCoreTemplate = createFeedbackControllerTemplate(controllableRigidBodies, 1);
      JointDesiredOutputList lowLevelControllerOutput = new JointDesiredOutputList(oneDoFJoints);
      return new WholeBodyControllerCore(toolbox, controllerCoreTemplate, lowLevelControllerOutput, registry);
   }

   /**
    * Convenience method to create the template necessary for the controller core to create all the
    * necessary feedback controllers.
    * 
    * @param controllableRigidBodies the collection of all the rigid-bodies that will be controllable
    *                                by the user. If it is {@code null}, then all the rigid-bodies of
    *                                the robot will be controllable.
    * @return the template for the controller core.
    */
   private FeedbackControllerTemplate createFeedbackControllerTemplate(Collection<? extends RigidBodyBasics> controllableRigidBodies,
                                                                       int numberOfControllersPerBody)
   {
      FeedbackControllerTemplate template = new FeedbackControllerTemplate();
      template.setAllowDynamicControllerConstruction(true);
      template.enableCenterOfMassFeedbackController();
      Collection<? extends RigidBodyBasics> rigidBodies;

      if (controllableRigidBodies != null)
         rigidBodies = controllableRigidBodies;
      else
         rigidBodies = rootBody.subtreeList();

      rigidBodies.stream().forEach(rigidBody -> template.enableSpatialFeedbackController(rigidBody, numberOfControllersPerBody));

      SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).forEach(template::enableOneDoFJointFeedbackController);
      return template;
   }

   /**
    * This marks the initialization phase. It is either called once when this toolbox wakes up or when
    * it is reinitialized.
    * <p>
    * It snaps {@code rootJoint} and {@code oneDoFJoints} to the most recent robot configuration
    * received from the walking controller. It also initializes the information needed to hold the
    * center of mass and support foot/feet in place.
    * </p>
    *
    * @return {@code true} if this controller is good to go and solve a new problem. It needs to have
    *         received at least once a robot configuration from the controller, otherwise this will
    *         fail and prevent the user from using this toolbox.
    */
   @Override
   public boolean initialize()
   {
      boolean success = initializeInternal();

      KinematicsToolboxOutputStatus status = new KinematicsToolboxOutputStatus();
      status.setJointNameHash(-1);
      status.setSolutionQuality(Double.NaN);
      status.setCurrentToolboxState(success ? CURRENT_TOOLBOX_STATE_INITIALIZE_SUCCESSFUL : CURRENT_TOOLBOX_STATE_INITIALIZE_FAILURE_MISSING_RCD);
      reportMessage(status);

      return success;
   }

   protected boolean initializeInternal()
   {
      threadTimer.clear();
      userFBCommands.clear();
      previousUserFBCommands.clear();
      isUserProvidingSupportPolygon.set(false);

      RobotConfigurationData robotConfigurationData = concurrentRobotConfigurationDataCopier.getCopyForReading();
      boolean hasRobotConfigurationData = robotConfigurationData != null;

      if (!hasRobotConfigurationData)
      {
         commandInputManager.clearAllCommands();
      }
      else
      {
         robotConfigurationDataInternal.set(robotConfigurationData);

         // Initializes this desired robot to the most recent robot configuration data received from the walking controller.
         KinematicsToolboxHelper.setRobotStateFromRobotConfigurationData(robotConfigurationDataInternal, rootJoint, oneDoFJoints);
         if (initialRobotConfigurationMap != null)
         {
            initialRobotConfigurationMap.forEachEntry((joint, q_priv) ->
            {
               joint.setQ(q_priv);
               return true;
            });
         }

         // Sets the privileged configuration to match the current robot configuration such that the solution will be as close as possible to the current robot configuration.
         snapPrivilegedConfigurationToCurrent();
         privilegedWeight.set(DEFAULT_PRIVILEGED_CONFIGURATION_WEIGHT);
         privilegedConfigurationGain.set(DEFAULT_PRIVILEGED_CONFIGURATION_GAIN);
         // It is required to update the tools now as it is only done at the end of each iteration.
         updateTools();
      }

      // By default, always constrain the center of mass according to the current support polygon (if defined).
      enableSupportPolygonConstraint.set(true);

      return hasRobotConfigurationData;
   }

   /**
    * This is the control loop called periodically only when the user requested a solution for a
    * desired set of inputs.
    * <p>
    * Each time this method is called, the multi-body system gets closer to the user inputs.
    * </p>
    */
   @Override
   public void updateInternal()
   {
      threadTimer.start();

      // Compiling all the commands to be submitted to the controller core.
      controllerCoreCommand.clear();

      FeedbackControlCommandBuffer feedbackControlCommandBuffer = controllerCoreCommand.getFeedbackControlCommandList();
      InverseKinematicsCommandBuffer inverseKinematicsCommandBuffer = controllerCoreCommand.getInverseKinematicsCommandList();
      consumeUserCommands(feedbackControlCommandBuffer, inverseKinematicsCommandBuffer);
      getAdditionalFeedbackControlCommands(feedbackControlCommandBuffer);

      inverseKinematicsCommandBuffer.addInverseKinematicsOptimizationSettingsCommand().set(activeOptimizationSettings);
      if (submitPrivilegedConfigurationCommand)
      {
         inverseKinematicsCommandBuffer.addPrivilegedConfigurationCommand().set(privilegedConfigurationCommand);
         submitPrivilegedConfigurationCommand = false;
      }
      getAdditionalInverseKinematicsCommands(inverseKinematicsCommandBuffer);
      computeCollisionCommands(collisionResults, inverseKinematicsCommandBuffer);

      // Save all commands used for this control tick for computing the solution quality.
      allFeedbackControlCommands.clear();
      allFeedbackControlCommands.addCommandList(feedbackControlCommandBuffer);

      if (minimizeAngularMomentum.getValue())
      {
         angularMomentumCommand.setWeight(angularMomentumWeight.getValue());
         inverseKinematicsCommandBuffer.addMomentumCommand().set(angularMomentumCommand);
      }

      /*
       * Submitting and requesting the controller core to run the feedback controllers, formulate and
       * solve the optimization problem for this control tick.
       */
      controllerCore.reset();
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();

      // Calculating the solution quality based on sum of all the active feedback controllers' output velocity.
      solutionQuality.set(solutionQualityCalculator.calculateSolutionQuality(feedbackControllerDataHolder, totalRobotMass, 1.0 / GLOBAL_PROPORTIONAL_GAIN));

      // Updating the the robot state from the current solution, initializing the next control tick.
      KinematicsToolboxHelper.setRobotStateFromControllerCoreOutput(controllerCore.getControllerCoreOutput(), rootJoint, oneDoFJoints);
      updateVisualization();

      inverseKinematicsSolution.setCurrentToolboxState(CURRENT_TOOLBOX_STATE_RUNNING);
      MessageTools.packDesiredJointState(inverseKinematicsSolution, rootJoint, oneDoFJoints);
      inverseKinematicsSolution.setSolutionQuality(solutionQuality.getDoubleValue());
      /*
       * Update tools for the next iteration. Only need to do it 1 per iteration and since it is updated
       * in the initialization method, it can be done at the end of the control tick. By doing this at the
       * end and computing collisions at the end, when visualizing the collisions they are exactly in sync
       * with the robot configuration.
       */
      updateTools();
      computeCollisions();

      double currentTime = Conversions.nanosecondsToSeconds(System.nanoTime());

      if (timeLastSolutionPublished.getValue() == 0.0 || currentTime - timeLastSolutionPublished.getValue() >= publishSolutionPeriod.getValue())
      {
         reportMessage(inverseKinematicsSolution);
         timeLastSolutionPublished.set(currentTime);
      }
      threadTimer.stop();
   }

   /**
    * Updates all the reference frames and the twist calculator. This method needs to be called at the
    * beginning of each control tick.
    */
   protected void updateTools()
   {
      rootBody.updateFramesRecursively();
      centerOfMassFrame.update();
   }

   /**
    * Checking if there is any new command available, in which case they polled from the
    * {@link #commandInputManager} and processed to update the state of the current optimization run.
    * 
    * @param ikCommandBufferToPack the buffer used to store the inverse kinematics commands to be
    *                              executed for this control tick.
    * @param fbCommandBufferToPack the buffer used to store the feedback control commands to be
    *                              executed for this control tick.
    */
   private void consumeUserCommands(FeedbackControlCommandBuffer fbCommandBufferToPack, InverseKinematicsCommandBuffer ikCommandBufferToPack)
   {
      consumeUserConfigurationCommands();
      consumeUserMotionObjectiveCommands(fbCommandBufferToPack, ikCommandBufferToPack);
      consumeUserContactStateCommands(ikCommandBufferToPack);
   }

   private void consumeUserConfigurationCommands()
   {
      if (commandInputManager.isNewCommandAvailable(KinematicsToolboxConfigurationCommand.class))
      {
         KinematicsToolboxConfigurationCommand command = commandInputManager.pollNewestCommand(KinematicsToolboxConfigurationCommand.class);

         if (command.getJointVelocityWeight() <= 0.0)
            activeOptimizationSettings.setJointVelocityWeight(optimizationSettings.getJointVelocityWeight());
         else
            activeOptimizationSettings.setJointVelocityWeight(command.getJointVelocityWeight());
         if (command.getJointAccelerationWeight() < 0.0)
            activeOptimizationSettings.setJointAccelerationWeight(optimizationSettings.getJointAccelerationWeight());
         else
            activeOptimizationSettings.setJointAccelerationWeight(command.getJointAccelerationWeight());

         if (command.getDisableCollisionAvoidance())
            enableCollisionAvoidance.set(false);
         if (command.getEnableCollisionAvoidance())
            enableCollisionAvoidance.set(true);
         if (command.getDisableJointVelocityLimits())
            activeOptimizationSettings.setJointVelocityLimitMode(JointVelocityLimitMode.DISABLED);
         if (command.getEnableJointVelocityLimits())
            activeOptimizationSettings.setJointVelocityLimitMode(JointVelocityLimitMode.ENABLED);
         if (command.getDisableInputPersistence())
            setPreserveUserCommandHistory(false);
         else if (command.getEnableInputPersistence())
            setPreserveUserCommandHistory(true);
         if (command.getEnableSupportPolygonConstraint())
            enableSupportPolygonConstraint.set(true);
         else if (command.getDisableSupportPolygonConstraint())
            enableSupportPolygonConstraint.set(false);
      }

      if (commandInputManager.isNewCommandAvailable(KinematicsToolboxPrivilegedConfigurationCommand.class))
      {
         KinematicsToolboxPrivilegedConfigurationCommand command = commandInputManager.pollNewestCommand(KinematicsToolboxPrivilegedConfigurationCommand.class);

         /*
          * If there is a new privileged configuration, the desired robot state is updated alongside with the
          * privileged configuration and the initial center of mass position and foot poses.
          */
         KinematicsToolboxHelper.setRobotStateFromPrivilegedConfigurationData(command, rootJoint);

         if (command.hasPrivilegedJointAngles() || command.hasPrivilegedRootJointPosition() || command.hasPrivilegedRootJointOrientation())
            robotConfigurationReinitialized();

         if (command.getPrivilegedWeight() < 0.0)
         {
            privilegedWeight.set(DEFAULT_PRIVILEGED_CONFIGURATION_WEIGHT);
         }
         else
         {
            privilegedWeight.set(command.getPrivilegedWeight());
            privilegedConfigurationCommand.setDefaultWeight(privilegedWeight.getValue());
            submitPrivilegedConfigurationCommand = true;
         }

         if (command.getPrivilegedGain() < 0.0)
         {
            privilegedConfigurationGain.set(DEFAULT_PRIVILEGED_CONFIGURATION_GAIN);
         }
         else
         {
            privilegedConfigurationGain.set(command.getPrivilegedGain());
            privilegedConfigurationCommand.setDefaultConfigurationGain(privilegedConfigurationGain.getValue());
            submitPrivilegedConfigurationCommand = true;
         }

         if (command.hasPrivilegedJointAngles())
            snapPrivilegedConfigurationToCurrent();
      }
   }

   private void consumeUserMotionObjectiveCommands(FeedbackControlCommandBuffer fbCommandBufferToPack, InverseKinematicsCommandBuffer ikCommandBufferToPack)
   {
      previousUserFBCommands.set(userFBCommands);
      userFBCommands.clear();

      // Keeping track of whether we received at least 1 command, if not we re-submit the previous commands regardless of preserveUserCommandHistory.
      boolean noCommandReceived = true;

      /***************************************
       * Individual center of mass commands
       ***************************************/
      if (commandInputManager.isNewCommandAvailable(KinematicsToolboxCenterOfMassCommand.class))
      {
         noCommandReceived = false;

         KinematicsToolboxCenterOfMassCommand command = commandInputManager.pollNewestCommand(KinematicsToolboxCenterOfMassCommand.class);
         KinematicsToolboxHelper.consumeCenterOfMassCommand(command, spatialGains.getPositionGains(), userFBCommands.addCenterOfMassFeedbackControlCommand());

         if (preserveUserCommandHistory.getValue())
         {
            // We override previous commands if any.
            while (!previousUserFBCommands.getCenterOfMassFeedbackControlCommandBuffer().isEmpty())
            {
               previousUserFBCommands.removeCommand(previousUserFBCommands.getCenterOfMassFeedbackControlCommandBuffer().get(0));
            }
         }
      }

      /***************************************
       * Individual rigid-body commands
       ***************************************/
      if (commandInputManager.isNewCommandAvailable(KinematicsToolboxRigidBodyCommand.class))
      {
         List<KinematicsToolboxRigidBodyCommand> commands = commandInputManager.pollNewCommands(KinematicsToolboxRigidBodyCommand.class);

         for (int i = 0; i < commands.size(); i++)
         {
            noCommandReceived = false;
            KinematicsToolboxRigidBodyCommand command = commands.get(i);
            RigidBodyBasics endEffector = command.getEndEffector();
            SpatialFeedbackControlCommand rigidBodyCommand = userFBCommands.addSpatialFeedbackControlCommand();
            KinematicsToolboxHelper.consumeRigidBodyCommand(command, rootBody, spatialGains, rigidBodyCommand);
            rigidBodyCommand.setPrimaryBase(getEndEffectorPrimaryBase(rigidBodyCommand.getEndEffector()));

            if (preserveUserCommandHistory.getValue())
            {
               // We override previous commands if any.
               for (int j = previousUserFBCommands.getSpatialFeedbackControlCommandBuffer().size() - 1; j >= 0; j--)
               {
                  SpatialFeedbackControlCommand previousCommand = previousUserFBCommands.getSpatialFeedbackControlCommandBuffer().get(j);
                  if (previousCommand.getEndEffector() == endEffector)
                     previousUserFBCommands.removeCommand(previousCommand);
               }
            }
         }
      }

      /***************************************
       * Individual 1 DoF joint commands
       ***************************************/
      if (commandInputManager.isNewCommandAvailable(KinematicsToolboxOneDoFJointCommand.class))
      {
         List<KinematicsToolboxOneDoFJointCommand> commands = commandInputManager.pollNewCommands(KinematicsToolboxOneDoFJointCommand.class);

         for (int i = 0; i < commands.size(); i++)
         {
            noCommandReceived = false;
            KinematicsToolboxOneDoFJointCommand command = commands.get(i);
            OneDoFJointBasics joint = command.getJoint();
            OneDoFJointFeedbackControlCommand jointCommand = userFBCommands.addOneDoFJointFeedbackControlCommand();
            KinematicsToolboxHelper.consumeJointCommand(command, jointGains, jointCommand);

            if (preserveUserCommandHistory.getValue())
            {
               // We override previous commands if any.
               for (int j = 0; j < previousUserFBCommands.getOneDoFJointFeedbackControlCommandBuffer().size(); j++)
               {
                  OneDoFJointFeedbackControlCommand previousCommand = previousUserFBCommands.getOneDoFJointFeedbackControlCommandBuffer().get(j);
                  if (previousCommand.getJoint() == joint)
                     previousUserFBCommands.removeCommand(previousCommand);
               }
            }
         }
      }

      /***************************************
       * Command collection
       ***************************************/
      if (commandInputManager.isNewCommandAvailable(KinematicsToolboxInputCollectionCommand.class))
      {
         KinematicsToolboxInputCollectionCommand command = commandInputManager.pollNewestCommand(KinematicsToolboxInputCollectionCommand.class);

         // CoM inputs
         RecyclingArrayList<KinematicsToolboxCenterOfMassCommand> centerOfMassInputs = command.getCenterOfMassInputs();
         for (int j = 0; j < centerOfMassInputs.size(); j++)
         {
            noCommandReceived = false;
            KinematicsToolboxCenterOfMassCommand input = centerOfMassInputs.get(j);
            KinematicsToolboxHelper.consumeCenterOfMassCommand(input, spatialGains.getPositionGains(), userFBCommands.addCenterOfMassFeedbackControlCommand());

            if (preserveUserCommandHistory.getValue())
            {
               // We override previous commands if any.
               while (!previousUserFBCommands.getCenterOfMassFeedbackControlCommandBuffer().isEmpty())
               {
                  previousUserFBCommands.removeCommand(previousUserFBCommands.getCenterOfMassFeedbackControlCommandBuffer().get(0));
               }
            }
         }

         // Rigid-body inputs
         RecyclingArrayList<KinematicsToolboxRigidBodyCommand> rigidBodyInputs = command.getRigidBodyInputs();
         for (int j = 0; j < rigidBodyInputs.size(); j++)
         {
            noCommandReceived = false;
            KinematicsToolboxRigidBodyCommand input = rigidBodyInputs.get(j);
            RigidBodyBasics endEffector = input.getEndEffector();
            SpatialFeedbackControlCommand rigidBodyCommand = userFBCommands.addSpatialFeedbackControlCommand();
            KinematicsToolboxHelper.consumeRigidBodyCommand(input, rootBody, spatialGains, rigidBodyCommand);
            rigidBodyCommand.setPrimaryBase(getEndEffectorPrimaryBase(rigidBodyCommand.getEndEffector()));

            if (preserveUserCommandHistory.getValue())
            {
               // We override previous commands if any.
               for (int k = previousUserFBCommands.getSpatialFeedbackControlCommandBuffer().size() - 1; k >= 0; k--)
               {
                  SpatialFeedbackControlCommand previousCommand = previousUserFBCommands.getSpatialFeedbackControlCommandBuffer().get(k);
                  if (previousCommand.getEndEffector() == endEffector)
                     previousUserFBCommands.removeCommand(previousCommand);
               }
            }
         }

         // 1-DoF joint inputs
         RecyclingArrayList<KinematicsToolboxOneDoFJointCommand> jointInputs = command.getJointInputs();
         for (int j = 0; j < jointInputs.size(); j++)
         {
            noCommandReceived = false;
            KinematicsToolboxOneDoFJointCommand input = jointInputs.get(j);
            OneDoFJointBasics joint = input.getJoint();
            OneDoFJointFeedbackControlCommand jointCommand = userFBCommands.addOneDoFJointFeedbackControlCommand();
            KinematicsToolboxHelper.consumeJointCommand(input, jointGains, jointCommand);

            if (preserveUserCommandHistory.getValue())
            {
               // We override previous commands if any.
               for (int k = 0; k < previousUserFBCommands.getOneDoFJointFeedbackControlCommandBuffer().size(); k++)
               {
                  OneDoFJointFeedbackControlCommand previousCommand = previousUserFBCommands.getOneDoFJointFeedbackControlCommandBuffer().get(k);
                  if (previousCommand.getJoint() == joint)
                     previousUserFBCommands.removeCommand(previousCommand);
               }
            }
         }

         // Contact state
         if (command.hasConstactStateInput())
         {
            KinematicsToolboxContactStateCommand contactStateInput = command.getContactStateInput();
            processUserContactStateCommand(contactStateInput, ikCommandBufferToPack);
         }
      }

      if (noCommandReceived)
      {
         fbCommandBufferToPack.addCommandList(previousUserFBCommands);
         numberOfActiveCommands.set(previousUserFBCommands.getNumberOfCommands());
         userFBCommands.set(previousUserFBCommands);
         previousUserFBCommands.clear();
      }
      else
      {
         if (preserveUserCommandHistory.getValue())
         { // We move the remaining previous commands to the current set of user commands.
            userFBCommands.addCommandList(previousUserFBCommands);
         }
         previousUserFBCommands.clear();
         // Add to the commandBufferToPack so it gets submitted to the controller core.
         fbCommandBufferToPack.addCommandList(userFBCommands);
         // Stats accessible from remote visualizer.
         numberOfActiveCommands.set(userFBCommands.getNumberOfCommands());
      }
   }

   private void consumeUserContactStateCommands(InverseKinematicsCommandBuffer bufferToPack)
   {
      if (commandInputManager.isNewCommandAvailable(KinematicsToolboxContactStateCommand.class))
      {
         KinematicsToolboxContactStateCommand command = commandInputManager.pollNewestCommand(KinematicsToolboxContactStateCommand.class);
         processUserContactStateCommand(command, bufferToPack);
      }
   }

   private void processUserContactStateCommand(KinematicsToolboxContactStateCommand command, InverseKinematicsCommandBuffer bufferToPack)
   {
      isUserProvidingSupportPolygon.set(command.getNumberOfContacts() > 0);
      if (command.getCenterOfMassMargin() >= 0.0)
         centerOfMassSafeMargin.set(command.getCenterOfMassMargin());

      contactPointLocations.clear();

      for (int i = 0; i < command.getNumberOfContacts(); i++)
      {
         contactPointLocations.add(command.getContactPoint(i).getPosition());
      }

      if (!contactPointLocations.isEmpty())
         updateSupportPolygonConstraint(contactPointLocations, bufferToPack);
   }

   protected void updateSupportPolygonConstraint(List<? extends FramePoint3DReadOnly> contactPoints, InverseKinematicsCommandBuffer bufferToPack)
   {
      if (!enableSupportPolygonConstraint.getValue())
         return;

      newSupportPolygon.clear(worldFrame);
      for (int i = 0; i < contactPoints.size(); i++)
      {
         newSupportPolygon.addVertexMatchingFrame(contactPoints.get(i));
      }
      newSupportPolygon.update();

      // If the support polygon is empty or too small, we don't apply the constraint, it would likely cause the QP to fail.
      if (newSupportPolygon.getNumberOfVertices() <= 2 || newSupportPolygon.getArea() < MathTools.square(0.01))
         return;

      if (!newSupportPolygon.epsilonEquals(supportPolygon, 5.0e-3))
      { // Update the polygon only if there is an actual update.
         supportPolygon.set(newSupportPolygon);
         convexPolygonScaler.scaleConvexPolygon(supportPolygon, centerOfMassSafeMargin.getValue(), shrunkConvexPolygon);
         shrunkSupportPolygonVertices.clear();
         for (int i = 0; i < shrunkConvexPolygon.getNumberOfVertices(); i++)
            shrunkSupportPolygonVertices.add().set(shrunkConvexPolygon.getVertex(i));

         for (int i = shrunkSupportPolygonVertices.size() - 1; i >= 0; i--)
         { // Filtering vertices that barely expand the polygon.
            Point2DReadOnly vertex = shrunkSupportPolygonVertices.get(i);
            Point2DReadOnly previousVertex = ListWrappingIndexTools.getPrevious(i, shrunkSupportPolygonVertices);
            Point2DReadOnly nextVertex = ListWrappingIndexTools.getNext(i, shrunkSupportPolygonVertices);

            if (EuclidGeometryTools.distanceFromPoint2DToLine2D(vertex, previousVertex, nextVertex) < 1.0e-3)
               shrunkSupportPolygonVertices.remove(i);
         }
      }

      centerOfMass.setToZero(centerOfMassFrame);
      centerOfMass.changeFrame(worldFrame);

      double distanceThreshold = 0.25 * centerOfMassSafeMargin.getValue();

      for (int i = 0; i < shrunkSupportPolygonVertices.size(); i++)
      { // Only adding constraints that are close to be violated.
         Point2DReadOnly vertex = shrunkSupportPolygonVertices.get(i);
         Point2DReadOnly nextVertex = ListWrappingIndexTools.getNext(i, shrunkSupportPolygonVertices);
         double signedDistanceToEdge = EuclidGeometryTools.signedDistanceFromPoint2DToLine2D(centerOfMass, vertex, nextVertex);

         if (signedDistanceToEdge > -distanceThreshold)
         {
            LinearMomentumConvexConstraint2DCommand command = bufferToPack.addLinearMomentumConvexConstraint2DCommand();
            command.clear();
            Vector2D h0 = command.addLinearMomentumConstraintVertex();
            Vector2D h1 = command.addLinearMomentumConstraintVertex();
            h0.sub(vertex, centerOfMass);
            h1.sub(nextVertex, centerOfMass);
            h0.scale(robotMass / updateDT);
            h1.scale(robotMass / updateDT);
         }
      }
   }

   /**
    * Evaluates the collision between each possible pair of collidables that can collide and stores the
    * result in {@link #collisionResults}.
    */
   private void computeCollisions()
   {
      collisionResults.clear();

      if (robotCollidables.isEmpty() || !enableCollisionAvoidance.getValue())
         return;

      int collisionIndex = 0;

      for (int collidableAIndex = 0; collidableAIndex < robotCollidables.size(); collidableAIndex++)
      {
         Collidable collidableA = robotCollidables.get(collidableAIndex);

         for (int collidableBIndex = collidableAIndex + 1; collidableBIndex < robotCollidables.size(); collidableBIndex++)
         {
            Collidable collidableB = robotCollidables.get(collidableBIndex);

            if (!collidableA.isCollidableWith(collidableB))
               continue;

            CollisionResult collisionResult = collisionResults.add();
            collidableA.evaluateCollision(collidableB, collisionResult);

            EuclidFrameShape3DCollisionResult collisionData = collisionResult.getCollisionData();

            if (collisionData.getSignedDistance() > collisionActivationDistanceThreshold.getValue())
               continue;

            if (collisionIndex < numberOfCollisionsToVisualize)
            {
               yoCollisionDistances[collisionIndex].set(collisionData.getSignedDistance());
               yoCollisionPointAs[collisionIndex].setMatchingFrame(collisionData.getPointOnA());
               yoCollisionPointBs[collisionIndex].setMatchingFrame(collisionData.getPointOnB());
            }

            collisionIndex++;
         }
      }
   }

   /**
    * Calculates and sets up the list of constraints to submit to the solver to prevent collisions.
    * 
    * @param collisions   the previously computed collisions.
    * @param bufferToPack buffer used to store the constraints to submit to the controller core.
    */
   public void computeCollisionCommands(List<CollisionResult> collisions, InverseKinematicsCommandBuffer bufferToPack)
   {
      if (collisions.isEmpty() || !enableCollisionAvoidance.getValue())
         return;

      int collisionIndex = 0;
      collisionFrames.clear();

      for (int i = 0; i < collisions.size(); i++)
      {
         CollisionResult collision = collisions.get(i);
         Collidable collidableA = collision.getCollidableA();
         Collidable collidableB = collision.getCollidableB();

         EuclidFrameShape3DCollisionResult collisionData = collision.getCollisionData();

         if (collisionData.getSignedDistance() > collisionActivationDistanceThreshold.getValue())
            continue;

         RigidBodyBasics bodyA = collidableA.getRigidBody();

         double sigma = -collisionData.getSignedDistance();
         double sigmaDot = sigma / updateDT;
         sigmaDot = Math.min(sigmaDot, maxCollisionResolutionVelocity.getValue());

         KinematicsCollisionFrame collisionFrame = collisionFrames.add();
         collisionFrame.update(collision, true);
         if (collisionIndex < numberOfCollisionsToVisualize)
            yoCollisionFramePoses[collisionIndex].setFromReferenceFrame(collisionFrame);

         SpatialVelocityCommand command = bufferToPack.addSpatialVelocityCommand();
         command.set(bodyA, collidableB.getRigidBody());
         command.getControlFramePose().setFromReferenceFrame(collisionFrame);
         SelectionMatrix6D selectionMatrix = command.getSelectionMatrix();
         selectionMatrix.clearSelection();
         selectionMatrix.selectLinearZ(true);
         selectionMatrix.setSelectionFrames(null, collisionFrame);

         command.setConstraintType(ConstraintType.GEQ_INEQUALITY);
         command.getDesiredLinearVelocity().setZ(sigmaDot);

         collisionIndex++;
      }
   }

   /**
    * Notifies when the user has sent a command that reinitializes the configuration of the robot.
    */
   protected void robotConfigurationReinitialized()
   {
      // Do nothing here
   }

   protected RigidBodyBasics getEndEffectorPrimaryBase(RigidBodyBasics endEffector)
   {
      return null;
   }

   private final List<FBPoint3D> rigidBodyPositions = new ArrayList<>();
   private final List<FBQuaternion3D> rigidBodyOrientations = new ArrayList<>();

   /**
    * Updates the graphic coordinate systems for the end-effectors that are actively controlled during
    * this control tick.
    */
   private void updateVisualization()
   {
      for (int i = 0; i < rigidBodiesWithVisualization.size(); i++)
      {
         RigidBodyBasics endEffector = rigidBodiesWithVisualization.get(i);
         YoGraphicCoordinateSystem coordinateSystem = desiredCoodinateSystems.get(endEffector);
         feedbackControllerDataHolder.getPositionData(endEffector, rigidBodyPositions, Type.DESIRED);
         if (rigidBodyPositions.isEmpty())
            coordinateSystem.hide();
         else // TODO Handle the case there are more than 1 active controller.
            coordinateSystem.setPosition(rigidBodyPositions.get(0));

         feedbackControllerDataHolder.getOrientationData(endEffector, rigidBodyOrientations, Type.DESIRED);
         if (rigidBodyOrientations.isEmpty())
            coordinateSystem.hide();
         else // TODO Handle the case there are more than 1 active controller.
            coordinateSystem.setOrientation(rigidBodyOrientations.get(0));
      }

      for (int i = 0; i < rigidBodiesWithVisualization.size(); i++)
      {
         RigidBodyBasics endEffector = rigidBodiesWithVisualization.get(i);
         YoGraphicCoordinateSystem coordinateSystem = currentCoodinateSystems.get(endEffector);
         feedbackControllerDataHolder.getPositionData(endEffector, rigidBodyPositions, Type.CURRENT);
         if (rigidBodyPositions.isEmpty())
            coordinateSystem.hide();
         else // TODO Handle the case there are more than 1 active controller.
            coordinateSystem.setPosition(rigidBodyPositions.get(0));

         feedbackControllerDataHolder.getOrientationData(endEffector, rigidBodyOrientations, Type.CURRENT);
         if (rigidBodyOrientations.isEmpty())
            coordinateSystem.hide();
         else // TODO Handle the case there are more than 1 active controller.
            coordinateSystem.setOrientation(rigidBodyOrientations.get(0));
      }
   }

   /**
    * Creates a {@code PrivilegedConfigurationCommand} to update the privileged joint angles to match
    * the current state of {@link #desiredFullRobotModel}.
    */
   private void snapPrivilegedConfigurationToCurrent()
   {
      privilegedConfigurationCommand.clear();
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         privilegedConfigurationCommand.addJoint(oneDoFJoints[i], oneDoFJoints[i].getQ());
      }
      privilegedConfigurationCommand.setDefaultWeight(privilegedWeight.getDoubleValue());
      privilegedConfigurationCommand.setDefaultConfigurationGain(privilegedConfigurationGain.getDoubleValue());
      privilegedConfigurationCommand.setDefaultMaxVelocity(privilegedMaxVelocity.getDoubleValue());
      submitPrivilegedConfigurationCommand = true;
   }

   public void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      concurrentRobotConfigurationDataCopier.getCopyForWriting().set(newConfigurationData);
      concurrentRobotConfigurationDataCopier.commit();
   }

   public boolean isUserControllingRigidBody(RigidBodyBasics rigidBody)
   {
      RecyclingArrayList<SpatialFeedbackControlCommand> currentFBCommands = userFBCommands.getSpatialFeedbackControlCommandBuffer();

      for (int i = 0; i < currentFBCommands.size(); i++)
      {
         if (currentFBCommands.get(i).getEndEffector() == rigidBody)
            return true;
      }

      RecyclingArrayList<SpatialFeedbackControlCommand> previousFBCommands = previousUserFBCommands.getSpatialFeedbackControlCommandBuffer();

      for (int i = 0; i < previousFBCommands.size(); i++)
      {
         if (previousFBCommands.get(i).getEndEffector() == rigidBody)
            return true;
      }

      return false;
   }

   public boolean isUserControllingJoint(OneDoFJointBasics joint)
   {
      RecyclingArrayList<OneDoFJointFeedbackControlCommand> currentFBCommands = userFBCommands.getOneDoFJointFeedbackControlCommandBuffer();

      for (int i = 0; i < currentFBCommands.size(); i++)
      {
         if (currentFBCommands.get(i).getJoint() == joint)
            return true;
      }

      RecyclingArrayList<OneDoFJointFeedbackControlCommand> previousFBCommands = previousUserFBCommands.getOneDoFJointFeedbackControlCommandBuffer();

      for (int i = 0; i < previousFBCommands.size(); i++)
      {
         if (previousFBCommands.get(i).getJoint() == joint)
            return true;
      }

      return false;
   }

   public boolean isUserControllingCenterOfMass()
   {
      return !userFBCommands.getCenterOfMassFeedbackControlCommandBuffer().isEmpty()
            || !previousUserFBCommands.getCenterOfMassFeedbackControlCommandBuffer().isEmpty();
   }

   public boolean isUserProvidingSupportPolygon()
   {
      return isUserProvidingSupportPolygon.getValue();
   }

   public YoPIDSE3Gains getDefaultSpatialGains()
   {
      return spatialGains;
   }

   public YoPIDGains getDefaultJointGains()
   {
      return jointGains;
   }

   protected void getAdditionalFeedbackControlCommands(FeedbackControlCommandBuffer bufferToPack)
   {
   }

   protected void getAdditionalInverseKinematicsCommands(InverseKinematicsCommandBuffer bufferToPack)
   {
   }

   @Override
   public boolean isDone()
   {
      // This toolbox should run until if falls asleep.
      return false;
   }

   public FloatingJointBasics getDesiredRootJoint()
   {
      return rootJoint;
   }

   public OneDoFJointBasics[] getDesiredOneDoFJoint()
   {
      return oneDoFJoints;
   }

   public Collection<? extends RigidBodyBasics> getControllableRigidBodies()
   {
      return controllableRigidBodies;
   }

   public KinematicsToolboxOutputStatus getSolution()
   {
      return inverseKinematicsSolution;
   }

   public FeedbackControllerDataHolderReadOnly getFeedbackControllerDataHolder()
   {
      return feedbackControllerDataHolder;
   }

   public void setPublishingSolutionPeriod(double periodInSeconds)
   {
      publishSolutionPeriod.set(periodInSeconds);
   }

   public void setPreserveUserCommandHistory(boolean value)
   {
      preserveUserCommandHistory.set(value);
   }

   public void minimizeAngularMomentum(boolean enable)
   {
      minimizeAngularMomentum.set(enable);
   }

   public double getUpdateDT()
   {
      return updateDT;
   }

   public YoDouble getCenterOfMassSafeMargin()
   {
      return centerOfMassSafeMargin;
   }

   public TObjectDoubleHashMap<OneDoFJointBasics> getInitialRobotConfigurationMap()
   {
      return initialRobotConfigurationMap;
   }
}