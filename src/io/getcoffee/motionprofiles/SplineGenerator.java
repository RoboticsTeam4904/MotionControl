package io.getcoffee.motionprofiles;


import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

strictfp public abstract class SplineGenerator {
	public static double INTEGRATION_GRANULARITY = 100;
	public LinkedList<SplineSegment> featureSegments;
	public Map<Double, Double> lengthPercentageTable;

	/**
	 * Generates an ordered list of distinct features of the spline. Distinct features are
	 * defined as a sudden change in curvature above the provided curve threshold.
	 * @param curveDerivativeThreshold
	 * @param granularity
	 * 
	 * @return ordered list of distinct features of the generated spline
	 */
	public void initialize(double curveDerivativeThreshold, double granularity) {
		featureSegments = new LinkedList<>();
		lengthPercentageTable = new HashMap<>();
		double lastPercentage = 0.0;
		// Hopefully the curvature is never non-zero at the initial position of the arc. (It really shouldn't be)
		double lastCurve = calcCurvature(0.0);
		double maxCurve = lastCurve;
		double maxCurveDerivative = 0.0;
		double absoluteArcSum = 0.0;
		double arcSum = 0.0;
		SplineSegment lastFeature = new SplineSegment(0);
		for (double i = 0; i < 1; i += 1 / granularity) {
			arcSum += calcSpeed(i);
			lengthPercentageTable.put(absoluteArcSum + arcSum, i);
			double instantCurve = calcCurvature(i);
			double instantCurveDerivative = Math.abs(lastCurve - instantCurve) * granularity;
			if(instantCurve > maxCurve) {
				maxCurve = instantCurve;
			}
			if(instantCurveDerivative > maxCurveDerivative) {
				maxCurveDerivative = instantCurveDerivative;
			}
			if (instantCurveDerivative > curveDerivativeThreshold) {
				lastPercentage = i;
				lastFeature = new SplineSegment(lastFeature.initCurve, instantCurve, maxCurve, maxCurveDerivative, arcSum);
				featureSegments.add(lastFeature);
				maxCurve = instantCurve;
				maxCurveDerivative = 0.0;
				absoluteArcSum += arcSum;
				arcSum = 0.0;
				lastFeature = new SplineSegment(instantCurve, lastPercentage);
			}
			lastCurve = instantCurve;
		}
		lastFeature = new SplineSegment(lastFeature.initCurve, calcCurvature(1),
				maxCurve, maxCurveDerivative, calcLength(lastPercentage, 1));
		featureSegments.add(lastFeature);
	}
	
	public void initialize(double curveThreshold) {
		initialize(curveThreshold, SplineGenerator.INTEGRATION_GRANULARITY);
	}

	/**
	 * Calculates the arc-length traveled given percentages a and b. Granularity
	 * determines the accuracy of the integration.
	 * 
	 * @param a
	 * @param b
	 * @param granularity
	 * @return
	 */
	public double calcLength(double a, double b, double granularity) {
		double arcSum = 0;
		for (double i = a; i < b; i += 1 / granularity) {
			arcSum += calcSpeed(i);
		}
		return arcSum / granularity;
	}

	/**
	 * Calculates the arc-length traveled given percentages a and b,
	 * with granularity set to a constant.
	 * 
	 * @param a
	 * @param b
	 * @return
	 */
	public double calcLength(double a, double b) {
		return calcLength(a, b, SplineGenerator.INTEGRATION_GRANULARITY);
	}

	protected double calcAbsoluteLength(double granularity) {
		return calcLength(0.0, 1.0, granularity);
	}

	protected Map<Integer, Map<Double, Double>> generatePercentageLengthTable(double granularity) {
		double arcSum = 0.0;
		for(double i = 0; i < 1; i += 1 / granularity) {

			arcSum += calcSpeed(i);
		}
	}

	protected double calcAbsoluteLength() {
		return calcLength(0.0, 1.0, SplineGenerator.INTEGRATION_GRANULARITY);
	}

	/**
	 * Equation for the curvature at a percentage of the arc-length.
	 * 
	 * @param s
	 *        the position along the spline from [0-1]
	 * @return the curvature of the spline at point s
	 */
	public double calcCurvature(double s) {
		Tuple<Double, Double> vel = calcVel(s);
		Tuple<Double, Double> acc = calcAcc(s);
		double speed = calcSpeed(s);
		// if speed == 0: use jerk and acc instead of acc and vel. if acc == 0, just return 0 or go deeper or something
		// System.out.println(speed);
		return (vel.getX() * acc.getY() - vel.getY() * acc.getX()) / (speed * speed * speed);
	}

	/**
	 * @param s
	 *        the position along the spline from [0-1]
	 * @return the 'velocity' (note that this is not true physical velocity but merely the magnitude of the spline velocity)
	 */
	public double calcSpeed(double s) {
		Tuple<Double, Double> vel = calcVel(s);
		return Math.sqrt(vel.getX() * vel.getX() + vel.getY() * vel.getY());
	}

	/**
	 * Initialize the position polynomial coefficients
	 */
	protected abstract void initializePos();

	/**
	 * @param s
	 *        the position along the spline from [0-1]
	 * @return a tuple of x-pos and y-pos
	 */
	public Tuple<Double, Double> calcPos(double s) {
		return new Tuple<>(PosX(s), PosY(s));
	}

	protected abstract double PosX(double s);

	protected abstract double PosY(double s);

	/**
	 * Initialize the velocity polynomial coefficients. The complexity for this is much
	 * simpler since you can use the position coefficients.
	 */
	protected abstract void initializeVel();

	public Tuple<Double, Double> calcVel(double s) {
		return new Tuple<>(VelX(s), VelY(s));
	}

	protected abstract double VelX(double s);

	protected abstract double VelY(double s);

	/**
	 * Initialize the acceleration polynomial coefficients.
	 */
	protected abstract void initializeAcc();

	public Tuple<Double, Double> calcAcc(double s) {
		return new Tuple<>(AccX(s), AccY(s));
	}

	protected abstract double AccX(double s);

	protected abstract double AccY(double s);
}