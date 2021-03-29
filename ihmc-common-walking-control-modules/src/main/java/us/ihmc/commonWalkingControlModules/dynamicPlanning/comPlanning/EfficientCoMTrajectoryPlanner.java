package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrixRMaj;
import org.ejml.data.DMatrixSparseCSC;
import org.ejml.interfaces.linsol.LinearSolverSparse;
import org.ejml.sparse.FillReducing;
import org.ejml.sparse.csc.factory.LinearSolverFactory_DSCC;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerTools.sufficientlyLongTime;

public class EfficientCoMTrajectoryPlanner implements CoMTrajectoryProvider
{
   private static boolean verbose = false;
   private static final int maxCapacity = 10;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   final DMatrixRMaj xCoefficientVector = new DMatrixRMaj(0, 1);
   final DMatrixRMaj yCoefficientVector = new DMatrixRMaj(0, 1);
   final DMatrixRMaj zCoefficientVector = new DMatrixRMaj(0, 1);

   private final DoubleProvider omega;
   private final YoDouble comHeight = new YoDouble("comHeightForPlanning", registry);
   private final double gravityZ;

   private final CoMTrajectoryPlannerIndexHandler indexHandler = new CoMTrajectoryPlannerIndexHandler();
   private final LinearCoMTrajectoryHandler trajectoryHandler = new LinearCoMTrajectoryHandler(registry);

   private final VRPTrajectorySubspaceSolver vrpTrajectorySubspaceSolver = new VRPTrajectorySubspaceSolver();

   private final FixedFramePoint3DBasics desiredCoMPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics desiredCoMVelocity = new FrameVector3D(worldFrame);
   private final FixedFrameVector3DBasics desiredCoMAcceleration = new FrameVector3D(worldFrame);

   private final FixedFramePoint3DBasics desiredDCMPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics desiredDCMVelocity = new FrameVector3D(worldFrame);

   private final FixedFramePoint3DBasics desiredVRPPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics desiredVRPVelocity = new FrameVector3D(worldFrame);
   private final FixedFramePoint3DBasics desiredECMPPosition = new FramePoint3D(worldFrame);

   private final RecyclingArrayList<FramePoint3D> startVRPPositions = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<FramePoint3D> endVRPPositions = new RecyclingArrayList<>(FramePoint3D::new);

   private final YoFramePoint3D finalDCMPosition = new YoFramePoint3D("goalDCMPosition", worldFrame, registry);

   private final YoFramePoint3D currentCoMPosition = new YoFramePoint3D("currentCoMPosition", worldFrame, registry);
   private final YoFrameVector3D currentCoMVelocity = new YoFrameVector3D("currentCoMVelocity", worldFrame, registry);

   private final RecyclingArrayList<FramePoint3D> dcmCornerPoints = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<FramePoint3D> comCornerPoints = new RecyclingArrayList<>(FramePoint3D::new);

   private final RecyclingArrayList<LineSegment3D> vrpSegments = new RecyclingArrayList<>(LineSegment3D::new);

   private final YoBoolean maintainInitialCoMVelocityContinuity = new YoBoolean("maintainInitialComVelocityContinuity", registry);

   private CornerPointViewer viewer = null;
   private BagOfBalls comTrajectoryViewer = null;

   private CoMContinuityCalculator comContinuityCalculator = null;

   public EfficientCoMTrajectoryPlanner(double gravityZ, double nominalCoMHeight, YoRegistry parentRegistry)
   {
      this.gravityZ = Math.abs(gravityZ);
      YoDouble omega = new YoDouble("omegaForPlanning", registry);

      comHeight.addListener(v -> omega.set(Math.sqrt(Math.abs(gravityZ) / comHeight.getDoubleValue())));
      comHeight.set(nominalCoMHeight);

      this.omega = omega;
      maintainInitialCoMVelocityContinuity.set(false);

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public EfficientCoMTrajectoryPlanner(double gravityZ, YoDouble omega, YoRegistry parentRegistry)
   {
      this.omega = omega;
      this.gravityZ = Math.abs(gravityZ);

      omega.addListener(v -> comHeight.set(gravityZ / MathTools.square(omega.getValue())));
      omega.notifyListeners();

      maintainInitialCoMVelocityContinuity.set(false);

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public void setMaintainInitialCoMVelocityContinuity(boolean maintainInitialCoMVelocityContinuity)
   {
      this.maintainInitialCoMVelocityContinuity.set(maintainInitialCoMVelocityContinuity);
   }

   public void setCornerPointViewer(CornerPointViewer viewer)
   {
      this.viewer = viewer;
   }

   public void setupCoMTrajectoryViewer(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      comTrajectoryViewer = new BagOfBalls(50, 0.01, YoAppearance.Black(), registry, yoGraphicsListRegistry);
   }

   public void setComContinuityCalculator(CoMContinuityCalculator comContinuityCalculator)
   {
      this.comContinuityCalculator = comContinuityCalculator;
   }

   public void reset()
   {
      trajectoryHandler.clearTrajectory();
   }

   public void initializeTrajectory(FramePoint3DReadOnly endPosition, double stepDuration)
   {
      stepDuration = Math.min(sufficientlyLongTime, stepDuration);
      trajectoryHandler.setLinear(currentCoMPosition, endPosition, omega.getValue(), stepDuration);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void setNominalCoMHeight(double nominalCoMHeight)
   {
      this.comHeight.set(nominalCoMHeight);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double getNominalCoMHeight()
   {
      return comHeight.getDoubleValue();
   }

   public double getOmega()
   {
      return omega.getValue();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void solveForTrajectory(List<? extends ContactStateProvider> contactSequence)
   {
      if (!ContactStateProviderTools.checkContactSequenceIsValid(contactSequence))
         throw new IllegalArgumentException("The contact sequence is not valid.");

      indexHandler.update(contactSequence);

      resetMatrices();

      CoMTrajectoryPlannerTools.computeVRPWaypoints(comHeight.getDoubleValue(),
                                                    gravityZ,
                                                    omega.getValue(),
                                                    currentCoMVelocity,
                                                    contactSequence,
                                                    startVRPPositions,
                                                    endVRPPositions,
                                                    true);

      solveForCoefficients(contactSequence);
      trajectoryHandler.setCoefficientsFromSolution(omega.getValue(), contactSequence, xCoefficientVector, yCoefficientVector, zCoefficientVector);

      if (maintainInitialCoMVelocityContinuity.getBooleanValue() && comContinuityCalculator != null)
      {
         int segmentId = comContinuityCalculator.getDepthForCalculation() - 1;
         double time = contactSequence.get(segmentId).getTimeInterval().getDuration();
         compute(segmentId,
                 time,
                 comPositionToThrowAway,
                 comVelocityToThrowAway,
                 comAccelerationToThrowAway,
                 dcmPositionToThrowAway,
                 dcmVelocityToThrowAway,
                 vrpStartPosition,
                 ecmpPositionToThrowAway);

         comContinuityCalculator.setInitialCoMPosition(currentCoMPosition);
         comContinuityCalculator.setInitialCoMVelocity(currentCoMVelocity);
         comContinuityCalculator.setFinalICPToAchieve(dcmPositionToThrowAway);

         if (comContinuityCalculator.solve(contactSequence))
         {
            comContinuityCalculator.getXCoefficientOverrides(xCoefficientVector);
            comContinuityCalculator.getYCoefficientOverrides(yCoefficientVector);
            comContinuityCalculator.getZCoefficientOverrides(zCoefficientVector);
         }
      }

      trajectoryHandler.setCoefficientsFromSolution(omega.getValue(), contactSequence, xCoefficientVector, yCoefficientVector, zCoefficientVector);

      updateCornerPoints(contactSequence);

      if (viewer != null)
      {
         viewer.updateDCMCornerPoints(dcmCornerPoints);
         viewer.updateCoMCornerPoints(comCornerPoints);
         viewer.updateVRPWaypoints(vrpSegments);
      }
      if (comTrajectoryViewer != null)
      {
         updateCoMTrajectoryViewer();
      }
   }

   private void solveForCoefficients(List<? extends ContactStateProvider> contactSequence)
   {

   }

   private final FramePoint3D comPositionToThrowAway = new FramePoint3D();
   private final FramePoint3D dcmPositionToThrowAway = new FramePoint3D();

   private final FrameVector3D comVelocityToThrowAway = new FrameVector3D();
   private final FrameVector3D comAccelerationToThrowAway = new FrameVector3D();
   private final FrameVector3D dcmVelocityToThrowAway = new FrameVector3D();
   private final FramePoint3D vrpStartPosition = new FramePoint3D();
   private final FramePoint3D vrpEndPosition = new FramePoint3D();
   private final FramePoint3D ecmpPositionToThrowAway = new FramePoint3D();

   private void updateCornerPoints(List<? extends ContactStateProvider> contactSequence)
   {
      comCornerPoints.clear();
      dcmCornerPoints.clear();
      vrpSegments.clear();

      boolean verboseBefore = verbose;
      verbose = false;
      for (int segmentId = 0; segmentId < Math.min(contactSequence.size(), maxCapacity + 1); segmentId++)
      {
         double duration = contactSequence.get(segmentId).getTimeInterval().getDuration();

         compute(segmentId,
                 0.0,
                 comCornerPoints.add(),
                 comVelocityToThrowAway,
                 comAccelerationToThrowAway,
                 dcmCornerPoints.add(),
                 dcmVelocityToThrowAway,
                 vrpStartPosition,
                 ecmpPositionToThrowAway);
         compute(segmentId,
                 duration,
                 comPositionToThrowAway,
                 comVelocityToThrowAway,
                 comAccelerationToThrowAway,
                 dcmPositionToThrowAway,
                 dcmVelocityToThrowAway,
                 vrpEndPosition,
                 ecmpPositionToThrowAway);

         vrpSegments.add().set(vrpStartPosition, vrpEndPosition);
      }

      verbose = verboseBefore;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void compute(int segmentId, double timeInPhase)
   {
      compute(segmentId,
              timeInPhase,
              desiredCoMPosition,
              desiredCoMVelocity,
              desiredCoMAcceleration,
              desiredDCMPosition,
              desiredDCMVelocity,
              desiredVRPPosition,
              desiredECMPPosition);

      if (verbose)
      {
         LogTools.info("At time " + timeInPhase + ", Desired DCM = " + desiredDCMPosition + ", Desired CoM = " + desiredCoMPosition);
      }
   }

   @Override
   public void compute(int segmentId,
                       double timeInPhase,
                       FixedFramePoint3DBasics comPositionToPack,
                       FixedFrameVector3DBasics comVelocityToPack,
                       FixedFrameVector3DBasics comAccelerationToPack,
                       FixedFramePoint3DBasics dcmPositionToPack,
                       FixedFrameVector3DBasics dcmVelocityToPack,
                       FixedFramePoint3DBasics vrpPositionToPack,
                       FixedFramePoint3DBasics ecmpPositionToPack)
   {
      if (segmentId < 0)
         throw new IllegalArgumentException("time is invalid.");

      trajectoryHandler.compute(segmentId,
                                timeInPhase,
                                comPositionToPack,
                                comVelocityToPack,
                                comAccelerationToPack,
                                dcmPositionToPack,
                                dcmVelocityToPack,
                                vrpPositionToPack,
                                desiredVRPVelocity);

      ecmpPositionToPack.set(vrpPositionToPack);
      ecmpPositionToPack.subZ(comHeight.getDoubleValue());
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void setInitialCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity)
   {
      this.currentCoMPosition.setMatchingFrame(centerOfMassPosition);
      this.currentCoMVelocity.setMatchingFrame(centerOfMassVelocity);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public FramePoint3DReadOnly getDesiredDCMPosition()
   {
      return desiredDCMPosition;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public FrameVector3DReadOnly getDesiredDCMVelocity()
   {
      return desiredDCMVelocity;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public FramePoint3DReadOnly getDesiredCoMPosition()
   {
      return desiredCoMPosition;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return desiredCoMVelocity;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public FrameVector3DReadOnly getDesiredCoMAcceleration()
   {
      return desiredCoMAcceleration;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public FramePoint3DReadOnly getDesiredVRPPosition()
   {
      return desiredVRPPosition;
   }

   public FrameVector3DReadOnly getDesiredVRPVelocity()
   {
      return desiredVRPVelocity;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public FramePoint3DReadOnly getDesiredECMPPosition()
   {
      return desiredECMPPosition;
   }

   private void updateCoMTrajectoryViewer()
   {
      comTrajectoryViewer.reset();

      boolean verboseBefore = verbose;
      verbose = false;
      for (int i = 0; i < comTrajectoryViewer.getNumberOfBalls(); i++)
      {
         double time = 0.05 * i;
         int segmentId = getSegmentNumber(time);
         double timeInSegment = getTimeInSegment(segmentId, time);

         compute(segmentId,
                 timeInSegment,
                 comPositionToThrowAway,
                 comVelocityToThrowAway,
                 comAccelerationToThrowAway,
                 dcmPositionToThrowAway,
                 dcmVelocityToThrowAway,
                 vrpStartPosition,
                 ecmpPositionToThrowAway);

         comTrajectoryViewer.setBall(comPositionToThrowAway);
      }

      verbose = verboseBefore;
   }

   /**
    * Resets and resizes the internal matrices.
    */
   private void resetMatrices()
   {
      int size = indexHandler.getTotalNumberOfCoefficients();

      xCoefficientVector.reshape(size, 1);
      yCoefficientVector.reshape(size, 1);
      zCoefficientVector.reshape(size, 1);

      xCoefficientVector.zero();
      yCoefficientVector.zero();
      zCoefficientVector.zero();
   }



   public boolean hasTrajectories()
   {
      return trajectoryHandler.hasTrajectory();
   }

   public void removeCompletedSegments(double timeToCrop)
   {
      trajectoryHandler.removeCompletedSegments(timeToCrop);
   }

   @Override
   public List<Polynomial3DReadOnly> getVRPTrajectories()
   {
      if (!hasTrajectories())
         throw new RuntimeException("VRP Trajectories are not calculated");

      return trajectoryHandler.getVrpTrajectories();
   }

   @Override
   public MultipleCoMSegmentTrajectoryGenerator getCoMTrajectory()
   {
      if (!hasTrajectories())
         throw new RuntimeException("CoM Trajectories are not calculated");

      return trajectoryHandler.getComTrajectory();
   }
}
