package io.getcoffee.motionprofiles;


import java.util.TreeMap;

strictfp public abstract class SplineGenerator {
	public static double INTEGRATION_GRANULARITY = 100;
	public TreeMap<Double, SplineSegment> featureSegmentMap = new TreeMap<>();

	/**
	 * Generates an ordered list of distinct features of the spline. Distinct features are
	 * defined as a sudden change in curvature above the provided curve threshold.
	 *
	 * @param curveDerivativeThreshold
	 * @param granularity
	 * 
	 * @return ordered list of distinct features of the generated spline
	 */
	public void initialize(double curveDerivativeThreshold, double granularity) {
		double lastPercentage = 0.0;
		// Hopefully the curvature is never non-zero at the initial position of the arc. (It really shouldn't be)
		double segmentCurve = calcCurvature(0.0);
		double lastCurve = calcCurvature(0.0);
		double maxSpeed = 0.0;
		double maxCurve = lastCurve;
		double maxCurveDerivative = 0.0;
		double absoluteArcSum = 0.0;
		double arcSum = 0.0;
		SplineSegment lastFeature = new SplineSegment(0);
		TreeMap<Double, SplinePoint> localLengthMap = new TreeMap<>();
		for (double i = 0; i < granularity; i++) {
			double percentage = i / granularity;
			double instantSpeed = calcSpeed(percentage);
			arcSum += instantSpeed;
			localLengthMap.put(absoluteArcSum, new SplinePoint(arcSum, percentage));
			double instantCurve = calcCurvature(percentage);
			double instantCurveDerivative = Math.abs(lastCurve - instantCurve) * granularity;
			if (instantSpeed > maxSpeed) {
				maxSpeed = instantSpeed;
			}
			if (instantCurve > maxCurve) {
				maxCurve = instantCurve;
			}
			if (instantCurveDerivative > maxCurveDerivative) {
				maxCurveDerivative = instantCurveDerivative;
			}
			double segmentCurveDerivative = Math.abs(segmentCurve - instantCurve);
			if (segmentCurveDerivative > curveDerivativeThreshold) {
				lastPercentage = percentage;
				lastFeature = new SplineSegment(lastFeature.initCurve, instantCurve, maxSpeed, maxCurve, maxCurveDerivative,
					arcSum,
					localLengthMap);
				featureSegmentMap.put(absoluteArcSum, lastFeature);
				maxCurve = instantCurve;
				maxSpeed = instantSpeed;
				maxCurveDerivative = 0.0;
				absoluteArcSum += arcSum;
				arcSum = 0.0;
				lastFeature = new SplineSegment(instantCurve);
				localLengthMap = new TreeMap<>();
				segmentCurve = instantCurve;
			}
			lastCurve = instantCurve;
		}
		lastFeature = new SplineSegment(lastFeature.initCurve, calcCurvature(1),
			maxCurve, maxCurveDerivative, calcLength(lastPercentage, 1), localLengthMap);
		featureSegmentMap.put(absoluteArcSum, lastFeature);
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
	 * Calculate a map from arclength (along segment) to s
	 * Alternatively also record the speed of the spline (depracated)
	 * 
	 * @param a
	 * @param b
	 * @param granularity
	 * @return
	 */
	public TreeMap<Double, Double> calcFeatureLengthMap(double a, double b, double granularity) {
		TreeMap<Double, Double> map = new TreeMap<Double, Double>();
		double length = 0;
		for (double i = a; i < b; i += 1 / granularity) {
			length += calcSpeed(i);
			map.put(length, i);
		}
		return map;
	}

	public TreeMap<Double, Double> calcFeatureLengthMap(double a, double b) {
		return calcFeatureLengthMap(a, b, INTEGRATION_GRANULARITY);
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
	 * Equation for the derivative of curvature at a percentage of the arc-length.
	 * 
	 * @see <a href="https://www.wolframalpha.com/input/?i=(x%27(t)+*+y%27%27(t)+-+y%27(t)+*+x%27%27(t))%2F(x%27(t)%5E2%2By%27(t)%5E2)%5E(3%2F2)">Wolfram-Alpha derivative</a>
	 *
	 * @param s
	 *        the position along the spline from [0-1]
	 * @return the derivature of curvature of the spline at point s
	 */
	public double calcCurvatureDerivative(double s) {
		Tuple<Double, Double> vel = calcVel(s);
		Tuple<Double, Double> acc = calcAcc(s);
		Tuple<Double, Double> jerk = calcJerk(s);
		return (1 / (2 * Math.pow(vel.getX() * vel.getX() + vel.getY() * vel.getY(), 5 / 2)))
			* (6 * (vel.getY() * acc.getX() - vel.getX() * acc.getY()) * (vel.getX() * acc.getX() + vel.getY() * acc.getY()) + 2
				* (vel.getX() * vel.getX() + vel.getY() * vel.getY()) * (-vel.getY() * jerk.getX() + vel.getX() * jerk.getY()));
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

	/**
	 * Initialize the jerk polynomial coefficients. For a cubic spline this should be the simplest.
	 */
	protected abstract void initializeJerk();

	public Tuple<Double, Double> calcJerk(double s) {
		return new Tuple<>(JerkX(s), JerkY(s));
	}

	protected abstract double JerkX(double s);

	protected abstract double JerkY(double s);
}