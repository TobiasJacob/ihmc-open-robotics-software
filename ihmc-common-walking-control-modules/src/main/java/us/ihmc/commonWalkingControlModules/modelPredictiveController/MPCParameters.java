package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class MPCParameters
{
   public static final boolean includeInitialCoMVelocityObjective = true;
   public static final boolean includeInitialCoMAccelerationObjective = true;
   public static final boolean includeFinalCoMPositionObjective = true;
   public static final boolean includeFinalCoMVelocityObjective = false;
   public static final boolean includeFinalDCMPositionObjective = true;

   public static final boolean includeRhoMinInequality = true;
   public static final boolean includeRhoMaxInequality = false;
   public static final boolean includeForceMinimization = false;
   public static final boolean includeRhoMinimization = true;
   public static final boolean includeRhoRateMinimization = true;

   public static final boolean includeIntermediateOrientationTracking = true;

   private static final double defaultMinRhoValue = 0.0;//05;

   public static final double defaultInitialComWeight = 5e2;
   public static final double defaultInitialComVelocityWeight = 1e2;
   public static final double defaultInitialComAccelerationWeight = 5e-3;
   public static final double defaultFinalComWeight = 1e1;
   public static final double defaultFinalVRPWeight = 1e-1;
   public static final double defaultFinalDCMWeight = 1e2;
   public static final double defaultVrpTrackingWeight = 1e2;
   public static final double defaultRhoTrackingWeight = 1e-5;
   public static final double defaultRhoRateTrackingWeight = 1e-6;
   public static final double defaultForceTrackingWeight = 1e-4;

   private static final double defaultOrientationAngleTrackingWeight = 1e-2;
   private static final double defaultOrientationVelocityTrackingWeight = 1e-6;

   private static final double defaultInitialOrientationWeight = 1e3;
   private static final double defaultFinalOrientationVelocityWeight = 1e1;
   private static final double defaultFinalOrientationAngleWeight = 1e2;

   public static final ConstraintType initialCoMPositionConstraintType = ConstraintType.OBJECTIVE;
   public static final ConstraintType initialCoMVelocityConstraintType = ConstraintType.OBJECTIVE;
   public static final ConstraintType finalCoMPositionConstraintType = ConstraintType.EQUALITY;
   public static final ConstraintType finalCoMVelocityConstraintType = ConstraintType.OBJECTIVE;
   public static final ConstraintType finalDCMPositionConstraintType = ConstraintType.OBJECTIVE;
   public static final ConstraintType finalVRPPositionConstraintType = ConstraintType.OBJECTIVE;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoDouble minRhoValue = new YoDouble("minRhoValue", registry);
   private final YoDouble initialComWeight = new YoDouble("initialComWeight", registry);
   private final YoDouble initialComVelocityWeight = new YoDouble("initialComVelocityWeight", registry);
   private final YoDouble initialComAccelerationWeight = new YoDouble("initialComAccelerationWeight", registry);
   private final YoDouble finalComWeight = new YoDouble("finalComWeight", registry);
   private final YoDouble finalVRPWeight = new YoDouble("finalVRPWeight", registry);
   private final YoDouble finalDCMWeight = new YoDouble("finalDCMWeight", registry);
   private final YoDouble vrpTrackingWeight = new YoDouble("vrpTrackingWeight", registry);
   private final YoDouble rhoTrackingWeight = new YoDouble("rhoTrackingWeight", registry);
   private final YoDouble rhoRateTrackingWeight = new YoDouble("rhoRateTrackingWeight", registry);
   private final YoDouble forceTrackingWeight = new YoDouble("forceTrackingWeight", registry);

   private final YoDouble orientationAngleTrackingWeight = new YoDouble("orientationAngleTrackingWeight", registry);
   private final YoDouble orientationVelocityTrackingWeight = new YoDouble("orientationVelocityTrackingWeight", registry);
   private final YoDouble initialOrientationWeight = new YoDouble("initialOrientationWeight", registry);
   private final YoDouble finalOrientationAngleWeight = new YoDouble("finalOrientationAngleWeight", registry);
   private final YoDouble finalOrientationVelocityWeight = new YoDouble("finalOrientationVelocityWeight", registry);

   public MPCParameters(YoRegistry parentRegistry)
   {
      minRhoValue.set(defaultMinRhoValue);
      initialComWeight.set(defaultInitialComWeight);
      initialComVelocityWeight.set(defaultInitialComVelocityWeight);
      initialComAccelerationWeight.set(defaultInitialComAccelerationWeight);
      finalComWeight.set(defaultFinalComWeight);
      finalVRPWeight.set(defaultFinalVRPWeight);
      finalDCMWeight.set(defaultFinalDCMWeight);
      vrpTrackingWeight.set(defaultVrpTrackingWeight);
      rhoTrackingWeight.set(defaultRhoTrackingWeight);
      rhoRateTrackingWeight.set(defaultRhoRateTrackingWeight);
      forceTrackingWeight.set(defaultForceTrackingWeight);

      orientationAngleTrackingWeight.set(defaultOrientationAngleTrackingWeight);
      orientationVelocityTrackingWeight.set(defaultOrientationVelocityTrackingWeight);
      initialOrientationWeight.set(defaultInitialOrientationWeight);
      finalOrientationAngleWeight.set(defaultFinalOrientationAngleWeight);
      finalOrientationVelocityWeight.set(defaultFinalOrientationVelocityWeight);

      parentRegistry.addChild(registry);
   }

   public boolean includeInitialCoMVelocityObjective()
   {
      return includeInitialCoMVelocityObjective;
   }

   public boolean includeInitialCoMAccelerationObjective()
   {
      return includeInitialCoMAccelerationObjective;
   }


   public boolean includeFinalCoMPositionObjective()
   {
      return includeFinalCoMPositionObjective;
   }

   public boolean includeFinalCoMVelocityObjective()
   {
      return includeFinalCoMVelocityObjective;
   }

   public boolean includeFinalDCMPositionObjective()
   {
      return includeFinalDCMPositionObjective;
   }

   public boolean includeRhoMinInequality()
   {
      return includeRhoMinInequality;
   }

   public boolean includeRhoMaxInequality()
   {
      return includeRhoMaxInequality;
   }

   public boolean includeForceMinimization()
   {
      return includeForceMinimization;
   }

   public boolean includeRhoMinimization()
   {
      return includeRhoMinimization;
   }

   public boolean includeRhoRateMinimization()
   {
      return includeRhoRateMinimization;
   }

   public boolean includeIntermediateOrientationTracking()
   {
      return includeIntermediateOrientationTracking;
   }

   public ConstraintType getInitialCoMPositionConstraintType()
   {
      return initialCoMPositionConstraintType;
   }

   public ConstraintType getInitialCoMVelocityConstraintType()
   {
      return initialCoMVelocityConstraintType;
   }

   public ConstraintType getFinalCoMPositionConstraintType()
   {
      return finalCoMPositionConstraintType;
   }

   public ConstraintType getFinalCoMVelocityConstraintType()
   {
      return finalCoMVelocityConstraintType;
   }

   public ConstraintType getFinalDCMPositionConstraintType()
   {
      return finalDCMPositionConstraintType;
   }

   public ConstraintType getFinalVRPPositionConstraintType()
   {
      return finalVRPPositionConstraintType;
   }

   public double getMinRhoValue()
   {
      return minRhoValue.getValue();
   }

   public double getInitialComWeight()
   {
      return initialComWeight.getValue();
   }

   public double getInitialComVelocityWeight()
   {
      return initialComVelocityWeight.getValue();
   }

   public double getInitialComAccelerationWeight()
   {
      return initialComAccelerationWeight.getValue();
   }

   public double getFinalComWeight()
   {
      return finalComWeight.getDoubleValue();
   }

   public double getFinalVRPWeight()
   {
      return finalVRPWeight.getDoubleValue();
   }

   public double getFinalDCMWeight()
   {
      return finalDCMWeight.getDoubleValue();
   }

   public double getVRPTrackingWeight()
   {
      return vrpTrackingWeight.getDoubleValue();
   }

   public double getRhoTrackingWeight()
   {
      return rhoTrackingWeight.getDoubleValue();
   }

   public double getRhoRateTrackingWeight()
   {
      return rhoRateTrackingWeight.getDoubleValue();
   }

   public double getForceTrackingWeight()
   {
      return forceTrackingWeight.getDoubleValue();
   }

   public double getInitialOrientationWeight()
   {
      return initialOrientationWeight.getDoubleValue();
   }

   public double getFinalOrientationAngleWeight()
   {
      return finalOrientationAngleWeight.getDoubleValue();
   }

   public double getFinalOrientationVelocityWeight()
   {
      return finalOrientationVelocityWeight.getDoubleValue();
   }

   public DoubleProvider getOrientationAngleTrackingWeightProvider()
   {
      return orientationAngleTrackingWeight;
   }

   public DoubleProvider getOrientationVelocityTrackingWeightProvider()
   {
      return orientationVelocityTrackingWeight;
   }
}
