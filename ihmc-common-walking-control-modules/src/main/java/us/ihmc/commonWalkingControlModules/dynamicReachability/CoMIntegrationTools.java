package us.ihmc.commonWalkingControlModules.dynamicReachability;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;

public class CoMIntegrationTools
{
   /**
    * Calculation of center of mass position at the end of a certain duration, assuming that the CMP
    * location is held constant throughout that time interval. This has an analytic solution to the
    * center of mass dynamics.
    *
    * Utilizes the same method as
    * {@link #integrateCoMPositionUsingConstantCMP(double, double, double, FramePoint3D, FramePoint3D, FramePoint3D, FramePoint3D)},
    * but assumes the initial time is 0.0.
    *
    * @param duration time interval over which to integrate the center of mass dynamics. Assumes
    *           that the initial time is time = 0.0.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param constantCMP constant location of the CMP during the integration interval.
    * @param initialICP initial location of the ICP at the beginning of the integration interval.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration.
    *           Modified.
    */
   public static void integrateCoMPositionUsingConstantCMP(double duration, double omega0, FramePoint3DReadOnly constantCMP, FramePoint3DReadOnly initialICP,
                                                           FramePoint3DReadOnly initialCoM, FramePoint3D finalCoMToPack)
   {
      integrateCoMPositionUsingConstantCMP(0.0, duration, omega0, constantCMP, initialICP, initialCoM, finalCoMToPack);
   }

   /**
    * Calculation of center of mass position at the end of a certain duration, assuming that the CMP
    * location is held constant throughout that time interval. This has an analytic solution to the
    * center of mass dynamics.
    *
    * Utilizes the same method as
    * {@link #integrateCoMPositionUsingConstantCMP(double, double, double, FramePoint3D, FramePoint3D, FramePoint3D, FramePoint3D)},
    * but assumes the initial time is 0.0.
    *
    * @param duration time interval over which to integrate the center of mass dynamics. Assumes
    *           that the initial time is time = 0.0.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param constantCMP constant location of the CMP during the integration interval.
    * @param initialICP initial location of the ICP at the beginning of the integration interval.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration.
    *           Modified.
    */
   public static void integrateCoMPositionUsingConstantCMP(double duration, double omega0, FramePoint3DReadOnly constantCMP, FramePoint3DReadOnly initialICP,
                                                           FramePoint2DReadOnly initialCoM, FramePoint2D finalCoMToPack)
   {
      integrateCoMPositionUsingConstantCMP(0.0, duration, omega0, constantCMP, initialICP, initialCoM, finalCoMToPack);
   }

   /**
    * Calculation of center of mass position at the end of a certain time interval, assuming that
    * the CMP location is held constant throughout that time interval. This has an analytic solution
    * to the center of mass dynamics.
    *
    * @param initialTime initial time of the interval over which to integrate.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param constantCMP constant location of the CMP during the integration interval.
    * @param initialICP initial location of the ICP at the beginning of the integration interval.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration.
    *           Modified.
    */
   public static void integrateCoMPositionUsingConstantCMP(double initialTime, double finalTime, double omega0, FramePoint3DReadOnly constantCMP,
                                                           FramePoint3DReadOnly initialICP, FramePoint3DReadOnly initialCoM, FramePoint3D finalCoMToPack)
   {
      initialCoM.checkReferenceFrameMatch(constantCMP);
      initialCoM.checkReferenceFrameMatch(initialICP);
      initialCoM.checkReferenceFrameMatch(finalCoMToPack);

      finalCoMToPack.set(initialCoM);

      double timeDelta = Math.max(finalTime - initialTime, 0.0);
      if (timeDelta == 0.0)
         return;

      double xPosition = integrateCoMPositionWithConstantCMP(timeDelta, omega0, initialICP.getX(), initialCoM.getX(), constantCMP.getX());
      double yPosition = integrateCoMPositionWithConstantCMP(timeDelta, omega0, initialICP.getY(), initialCoM.getY(), constantCMP.getY());

      finalCoMToPack.setX(xPosition);
      finalCoMToPack.setY(yPosition);
   }

   /**
    * Calculation of center of mass position at the end of a certain time interval, assuming that
    * the CMP location is held constant throughout that time interval. This has an analytic solution
    * to the center of mass dynamics.
    *
    * @param initialTime initial time of the interval over which to integrate.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param constantCMP constant location of the CMP during the integration interval.
    * @param initialICP initial location of the ICP at the beginning of the integration interval.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration.
    *           Modified.
    */
   public static void integrateCoMPositionUsingConstantCMP(double initialTime, double finalTime, double omega0, FramePoint3DReadOnly constantCMP,
                                                           FramePoint3DReadOnly initialICP, FramePoint2DReadOnly initialCoM, FramePoint2D finalCoMToPack)
   {
      initialCoM.checkReferenceFrameMatch(constantCMP);
      initialCoM.checkReferenceFrameMatch(initialICP);
      initialCoM.checkReferenceFrameMatch(finalCoMToPack);

      finalCoMToPack.set(initialCoM);

      double timeDelta = Math.max(finalTime - initialTime, 0.0);

      if (timeDelta == 0.0)
         return;

      double xPosition = integrateCoMPositionWithConstantCMP(timeDelta, omega0, initialICP.getX(), initialCoM.getX(), constantCMP.getX());
      double yPosition = integrateCoMPositionWithConstantCMP(timeDelta, omega0, initialICP.getY(), initialCoM.getY(), constantCMP.getY());

      finalCoMToPack.setX(xPosition);
      finalCoMToPack.setY(yPosition);
   }

   private static double integrateCoMPositionWithConstantCMP(double duration, double omega0, double initialICPPosition, double initialCoMPosition,
                                                             double cmpPosition)
   {
      if (duration == 0.0)
         return initialCoMPosition;

      double position = 0.5 * Math.exp(omega0 * duration) * (initialICPPosition - cmpPosition);
      position += Math.exp(-omega0 * duration) * (initialCoMPosition - 0.5 * (initialICPPosition + cmpPosition));
      position += cmpPosition;

      return position;
   }

   /**
    * Calculation of center of mass position at the end of a certain duration, assuming that the ICP
    * trajectory is in the form of a cubic spline over that time interval.
    *
    * Utilizes the same method as
    * {@link #integrateCoMPositionUsingCubicICP(double, double, double, double, ReferenceFrame, YoPolynomial, YoPolynomial, FramePoint3D, FramePoint3D)},
    * but assumes the initial time is 0.0, and segment duration is the integration duration.
    *
    * @param duration time interval over which to integrate the center of mass dynamics. Assumes
    *           that the initial time is time = 0.0, and that this is the entire duration of the
    *           spline.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param polynomialFrame reference frame in which the polynomial coefficients are defined.
    * @param xPolynomial polynomial value that defines the coefficients for the ICP trajectory in
    *           the X-Direction.
    * @param yPolynomial polynomial value that defines the coefficients for the ICP trajectory in
    *           the Y-Direction.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration.
    *           Modified.
    */
   public static void integrateCoMPositionFromCubicICP(double duration, double omega0, ReferenceFrame polynomialFrame, YoPolynomial xPolynomial,
                                                       YoPolynomial yPolynomial, FramePoint3DReadOnly initialCoM, FramePoint3D finalCoMToPack)
   {
      integrateCoMPositionFromCubicICP(duration, omega0, polynomialFrame, xPolynomial, yPolynomial, initialCoM, finalCoMToPack);
   }

   /**
    * Calculation of center of mass position at the end of a certain time, assuming that the ICP
    * trajectory is in the form of a cubic spline over that time interval.
    *
    * Utilizes the same method as
    * {@link #integrateCoMPositionUsingCubicICP(double, double, double, double, ReferenceFrame, YoPolynomial, YoPolynomial, FramePoint3D, FramePoint3D)},
    * but assumes the initial time is 0.0.
    *
    * @param time time interval over which to integrate the center of mass dynamics. Assumes that
    *           the initial time is time = 0.0
    * @param duration total duration for which the spline is defined.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param polynomialFrame reference frame in which the polynomial coefficients are defined.
    * @param xPolynomial polynomial value that defines the coefficients for the ICP trajectory in
    *           the X-Direction.
    * @param yPolynomial polynomial value that defines the coefficients for the ICP trajectory in
    *           the Y-Direction.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration.
    *           Modified.
    */
   public static void integrateCoMPositionUsingCubicICP(double time, double duration, double omega0, ReferenceFrame polynomialFrame, YoPolynomial xPolynomial,
                                                        YoPolynomial yPolynomial, FramePoint3DReadOnly initialCoM, FramePoint3D finalCoMToPack)
   {
      integrateCoMPositionUsingCubicICP(0.0, MathTools.clamp(time, 0.0, duration), duration, omega0, polynomialFrame, xPolynomial, yPolynomial, initialCoM,
                                        finalCoMToPack);
   }

   /**
    * Calculation of center of mass position at the end of a certain time, assuming that the ICP
    * trajectory is in the form of a cubic spline over that time interval.
    *
    * Utilizes the same method as
    * {@link #integrateCoMPositionUsingCubicICP(double, double, double, double, ReferenceFrame, YoPolynomial, YoPolynomial, FramePoint3D, FramePoint3D)},
    * but assumes the initial time is 0.0.
    *
    * @param time time interval over which to integrate the center of mass dynamics. Assumes that
    *           the initial time is time = 0.0
    * @param duration total duration for which the spline is defined.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param polynomialFrame reference frame in which the polynomial coefficients are defined.
    * @param xPolynomial polynomial value that defines the coefficients for the ICP trajectory in
    *           the X-Direction.
    * @param yPolynomial polynomial value that defines the coefficients for the ICP trajectory in
    *           the Y-Direction.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration.
    *           Modified.
    */
   public static void integrateCoMPositionUsingCubicICP(double time, double duration, double omega0, ReferenceFrame polynomialFrame, YoPolynomial xPolynomial,
                                                        YoPolynomial yPolynomial, FramePoint2DReadOnly initialCoM, FramePoint2D finalCoMToPack)
   {
      integrateCoMPositionUsingCubicICP(0.0, MathTools.clamp(time, 0.0, duration), duration, omega0, polynomialFrame, xPolynomial, yPolynomial, initialCoM,
                                        finalCoMToPack);
   }

   /**
    * Calculation of center of mass position at the end of certain time interval, assuming that the
    * ICP trajectory is in the form of a cubic spline over that time interval.
    *
    * @param initialTime time at the beginning of the time interval
    * @param finalTime time at the end of the time interval
    * @param duration total duration for which the spline is defined.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param polynomialFrame reference frame in which the polynomial coefficients are defined.
    * @param xPolynomial polynomial value that defines the coefficients for the ICP trajectory in
    *           the X-Direction.
    * @param yPolynomial polynomial value that defines the coefficients for the ICP trajectory in
    *           the Y-Direction.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration.
    *           Modified.
    */
   public static void integrateCoMPositionUsingCubicICP(double initialTime, double finalTime, double duration, double omega0, ReferenceFrame polynomialFrame,
                                                        YoPolynomial xPolynomial, YoPolynomial yPolynomial, FramePoint3DReadOnly initialCoM,
                                                        FramePoint3D finalCoMToPack)
   {
      initialCoM.checkReferenceFrameMatch(polynomialFrame);
      initialCoM.checkReferenceFrameMatch(finalCoMToPack);

      if (xPolynomial.getNumberOfCoefficients() != yPolynomial.getNumberOfCoefficients() && yPolynomial.getNumberOfCoefficients() != 4)
         throw new RuntimeException("The number of coefficients in the polynomials are wrong!");

      double integrationDuration = finalTime - initialTime;
      integrationDuration = MathTools.clamp(integrationDuration, 0.0, duration);

      finalCoMToPack.set(initialCoM);

      if (integrationDuration == 0.0)
         return;

      double xPosition = integrateCoMPositionOverPolynomial(initialCoM.getX(), integrationDuration, omega0, xPolynomial);
      double yPosition = integrateCoMPositionOverPolynomial(initialCoM.getY(), integrationDuration, omega0, yPolynomial);

      finalCoMToPack.setX(xPosition);
      finalCoMToPack.setY(yPosition);
   }

   /**
    * Calculation of center of mass position at the end of certain time interval, assuming that the
    * ICP trajectory is in the form of a cubic spline over that time interval.
    *
    * @param initialTime time at the beginning of the time interval
    * @param finalTime time at the end of the time interval
    * @param duration total duration for which the spline is defined.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param polynomialFrame reference frame in which the polynomial coefficients are defined.
    * @param xPolynomial polynomial value that defines the coefficients for the ICP trajectory in
    *           the X-Direction.
    * @param yPolynomial polynomial value that defines the coefficients for the ICP trajectory in
    *           the Y-Direction.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration.
    *           Modified.
    */
   public static void integrateCoMPositionUsingCubicICP(double initialTime, double finalTime, double duration, double omega0, ReferenceFrame polynomialFrame,
                                                        YoPolynomial xPolynomial, YoPolynomial yPolynomial, FramePoint2DReadOnly initialCoM,
                                                        FramePoint2D finalCoMToPack)
   {
      initialCoM.checkReferenceFrameMatch(polynomialFrame);
      initialCoM.checkReferenceFrameMatch(finalCoMToPack);

      if (xPolynomial.getNumberOfCoefficients() != yPolynomial.getNumberOfCoefficients() && yPolynomial.getNumberOfCoefficients() != 4)
         throw new RuntimeException("The number of coefficients in the polynomials are wrong!");

      double integrationDuration = finalTime - initialTime;
      integrationDuration = MathTools.clamp(integrationDuration, 0.0, duration);
      finalCoMToPack.set(initialCoM);

      if (integrationDuration == 0.0)
         return;

      double xPosition = integrateCoMPositionOverPolynomial(initialCoM.getX(), integrationDuration, omega0, xPolynomial);
      double yPosition = integrateCoMPositionOverPolynomial(initialCoM.getY(), integrationDuration, omega0, yPolynomial);

      finalCoMToPack.setX(xPosition);
      finalCoMToPack.setY(yPosition);
   }

   public static void integrateFinalCoMPositionFromCubicDCM(double segmentDuration, double omega0, ReferenceFrame polynomialFrame, YoPolynomial xPolynomial,
                                                            YoPolynomial yPolynomial, YoPolynomial zPolynomial, FramePoint3DReadOnly initialCoM,
                                                            FramePoint3D finalCoMToPack)
   {
      integrateCoMPositionUsingCubicDCM(0.0, segmentDuration, segmentDuration, omega0, polynomialFrame, xPolynomial, yPolynomial, zPolynomial, initialCoM,
                                        finalCoMToPack);
   }

   public static void integrateCoMPositionUsingCubicDCM(double initialTime, double finalTime, double segmentDuration, double omega0,
                                                        ReferenceFrame polynomialFrame, YoPolynomial xPolynomial, YoPolynomial yPolynomial,
                                                        YoPolynomial zPolynomial, FramePoint3DReadOnly initialCoM, FramePoint3D finalCoMToPack)
   {
      initialCoM.checkReferenceFrameMatch(polynomialFrame);
      initialCoM.checkReferenceFrameMatch(finalCoMToPack);

      if (xPolynomial.getNumberOfCoefficients() != yPolynomial.getNumberOfCoefficients()
            && yPolynomial.getNumberOfCoefficients() != zPolynomial.getNumberOfCoefficients() && zPolynomial.getNumberOfCoefficients() != 4)
         throw new RuntimeException("The number of coefficients in the polynomials are wrong!");

      double integrationDuration = finalTime - initialTime;
      integrationDuration = MathTools.clamp(integrationDuration, 0.0, segmentDuration);

      double xPosition = integrateCoMPositionOverPolynomial(initialCoM.getX(), integrationDuration, omega0, xPolynomial);
      double yPosition = integrateCoMPositionOverPolynomial(initialCoM.getY(), integrationDuration, omega0, yPolynomial);
      double zPosition = integrateCoMPositionOverPolynomial(initialCoM.getZ(), integrationDuration, omega0, zPolynomial);

      finalCoMToPack.setToZero(initialCoM.getReferenceFrame());
      finalCoMToPack.set(xPosition, yPosition, zPosition);
   }

   private static double integrateCoMPositionOverPolynomial(double initialPosition, double integrationDuration, double omega0, YoPolynomial polynomial)
   {
      if (integrationDuration == 0.0)
         return initialPosition;

      double position = polynomial.getCoefficient(3) * Math.pow(integrationDuration, 3.0);
      position += (polynomial.getCoefficient(2) + -3.0 / omega0 * polynomial.getCoefficient(3)) * Math.pow(integrationDuration, 2.0);
      position += (polynomial.getCoefficient(1) - 2.0 / omega0 * polynomial.getCoefficient(2) + 6.0 / Math.pow(omega0, 2.0) * polynomial.getCoefficient(3))
            * integrationDuration;
      position += (polynomial.getCoefficient(0) - 1.0 / omega0 * polynomial.getCoefficient(1) + 2.0 / Math.pow(omega0, 2.0) * polynomial.getCoefficient(2)
            - 6.0 / Math.pow(omega0, 3.0) * polynomial.getCoefficient(3));
      position += Math.exp(-omega0 * integrationDuration) * (initialPosition - polynomial.getCoefficient(0) + 1.0 / omega0 * polynomial.getCoefficient(1)
            - 2.0 / Math.pow(omega0, 2.0) * polynomial.getCoefficient(2) + 6.0 / Math.pow(omega0, 3.0) * polynomial.getCoefficient(3));

      return position;
   }

}
