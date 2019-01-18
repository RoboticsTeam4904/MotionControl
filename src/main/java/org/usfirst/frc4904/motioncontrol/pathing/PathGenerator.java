package org.usfirst.frc4904.motioncontrol.pathing;

import java.util.TreeMap;

import org.usfirst.frc4904.motioncontrol.MotionTrajectoryExecutor;
import org.usfirst.frc4904.motioncontrol.Tuple;

strictfp public abstract class PathGenerator {
	public static final double INTEGRATION_GRANULARITY = 100;
	public static final double CURVATURE_THRESHOLD = 0.2;
	public static double robotMaxAccel = MotionTrajectoryExecutor.robotMaxAccel;
	public static double robotMaxVel = MotionTrajectoryExecutor.robotMaxVel;
	public static double plantWidth = MotionTrajectoryExecutor.plantWidth;
	public TreeMap<Double, PathSegment> featureSegmentMap = new TreeMap<>();
	protected double[] PosX, PosY, VelX, VelY, AccX, AccY, JerkX, JerkY;
	public double absoluteLength;

	/**
	 * Generates an ordered list of distinct features of the spline. Distinct
	 * features are defined as a sudden change in curvature above the provided
	 * curve threshold.
	 *
	 * @param curveDerivativeThreshold
	 * @param granularity
	 *
	 * @return ordered list of distinct features of the generated spline
	 */
	public void initialize(double curveDerivativeThreshold, double granularity) {
		// absoluteLength = calcAbsoluteLength();
		// Hopefully the curvature is never non-zero at the initial position of
		// the arc. (It really shouldn't be)
		double segmentCurve = calcCurvature(0.0);
		double lastCurve = calcCurvature(0.0);
		double maxSpeed = 0.0;
		double maxCurve = lastCurve;
		double maxCurveDerivative = 0.0;
		double minAcc = -robotMaxAccel;
		double maxAcc = robotMaxAccel;
		double absoluteArcSum = 0.0;
		double arcSum = 0.0;
		PathSegment lastFeature = new PathSegment(segmentCurve);
		TreeMap<Double, Double> localLengthMap = new TreeMap<>();
		for (double i = 0; i < granularity; i++) {
			double percentage = i / granularity;
			double instantSpeed = calcSpeed(percentage);
			arcSum += instantSpeed / granularity;
			localLengthMap.put(arcSum, percentage);
			double instantCurve = calcCurvature(percentage);
			double instantCurveDerivative = Math.abs(lastCurve - instantCurve) * granularity;
			double k = Math.signum(instantCurve);
			double divisor = 1 + k * plantWidth * instantCurve;
			double o = Math.signum(k * instantCurveDerivative / divisor);
			double minVelSqrd;
			double maxVelSqrd;
			if (o == 1.0) {
				minVelSqrd = 0;
				maxVelSqrd = robotMaxVel * robotMaxVel;
			} else {
				maxVelSqrd = 0;
				minVelSqrd = robotMaxVel * robotMaxVel;
			}
			double instantMinAcc = (-robotMaxAccel
					- k * plantWidth * maxVelSqrd * instantCurveDerivative / instantSpeed) / divisor;
			double instantMaxAcc = (robotMaxAccel - k * plantWidth * minVelSqrd * instantCurveDerivative / instantSpeed)
					/ divisor;
			if (instantSpeed > maxSpeed) {
				maxSpeed = instantSpeed;
			}
			if (instantCurve > maxCurve) {
				maxCurve = instantCurve;
			}
			if (instantCurveDerivative > maxCurveDerivative) {
				maxCurveDerivative = instantCurveDerivative;
			}
			if (instantMinAcc > minAcc) {
				minAcc = instantMinAcc;
			}
			if (instantMaxAcc < maxAcc) {
				maxAcc = instantMaxAcc;
			}
			double segmentCurveDerivative = Math.abs(segmentCurve - instantCurve);
			if (segmentCurveDerivative > curveDerivativeThreshold) {
				lastFeature = new PathSegment(lastFeature.finCurve, instantCurve, maxCurve, maxCurveDerivative,
						maxSpeed, minAcc, maxAcc, arcSum, localLengthMap);
				featureSegmentMap.put(absoluteArcSum, lastFeature);
				maxCurve = instantCurve;
				maxSpeed = instantSpeed;
				minAcc = -robotMaxAccel;
				maxAcc = robotMaxAccel;
				maxCurveDerivative = 0.0;
				absoluteArcSum += arcSum;
				arcSum = 0.0;
				localLengthMap = new TreeMap<>();
				segmentCurve = instantCurve;
			}
			lastCurve = instantCurve;
		}
		lastFeature = new PathSegment(lastFeature.finCurve, calcCurvature(1), maxCurve, maxCurveDerivative, maxSpeed,
				minAcc, maxAcc, arcSum, localLengthMap);
		featureSegmentMap.put(absoluteArcSum, lastFeature);
	}

	protected void initialize(double threshold) {
		initialize(threshold, INTEGRATION_GRANULARITY);
	}

	protected void initialize() {
		initialize(CURVATURE_THRESHOLD, INTEGRATION_GRANULARITY);
	}

	/**
	 * Calculate a map from arclength (along segment) to s Alternatively also
	 * record the speed of the spline (deprecated)
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
	 * Calculates the arc-length traveled given percentages a and b, with
	 * granularity set to a constant.
	 * 
	 * @param a
	 * @param b
	 * @return
	 */
	/**
	 * Equation for the curvature at a percentage of the arc-length.
	 *
	 * @param s
	 *            the position along the spline from [0-1]
	 * @return the curvature of the spline at point s
	 */
	public double calcCurvature(double s) {
		Tuple<Double, Double> vel = calcVel(s);
		Tuple<Double, Double> acc = calcAcc(s);
		double speed = calcSpeed(s);
		// if speed == 0: use jerk and acc instead of acc and vel. if acc == 0,
		// just return 0 or go deeper or something
		// System.out.println(speed);
		return (vel.getX() * acc.getY() - vel.getY() * acc.getX()) / (speed * speed * speed);
	}

	/**
	 * Equation for the derivative of curvature at a percentage of the
	 * arc-length.
	 *
	 * @see <a href=
	 *      "https://www.wolframalpha.com/input/?i=(x%27(t)+*+y%27%27(t)+-+y%27(t)+*+x%27%27(t))%2F(x%27(t)%5E2%2By%27(t)%5E2)%5E(3%2F2)">Wolfram-Alpha
	 *      derivative</a>
	 *
	 * @param s
	 *            the position along the spline from [0-1]
	 * @return the derivative of curvature of the spline at point s
	 */
	public double calcCurvatureDerivative(double s) {
		Tuple<Double, Double> vel = calcVel(s);
		Tuple<Double, Double> acc = calcAcc(s);
		Tuple<Double, Double> jerk = calcJerk(s);
		return (1 / (2 * Math.pow(vel.getX() * vel.getX() + vel.getY() * vel.getY(), 5 / 2)))
				* (6 * (vel.getY() * acc.getX() - vel.getX() * acc.getY())
						* (vel.getX() * acc.getX() + vel.getY() * acc.getY())
						+ 2 * (vel.getX() * vel.getX() + vel.getY() * vel.getY())
								* (-vel.getY() * jerk.getX() + vel.getX() * jerk.getY()));
	}

	/**
	 * @param s
	 *            the position along the spline from [0-1]
	 * @return the 'velocity' (note that this is not true physical velocity but
	 *         merely the magnitude of the spline velocity)
	 */
	public double calcSpeed(double s) {
		Tuple<Double, Double> vel = calcVel(s);
		return Math.sqrt(vel.getX() * vel.getX() + vel.getY() * vel.getY());
	}

	/**
	 * @param s
	 *            the position along the spline from [0-1]
	 * @return a tuple of x-pos and y-pos
	 */
	public Tuple<Double, Double> calcPos(double s) {
		return new Tuple<>(PosX(s), PosY(s));
	}

	protected abstract double PosX(double s);

	protected abstract double PosY(double s);

	public Tuple<Double, Double> calcVel(double s) {
		return new Tuple<>(VelX(s), VelY(s));
	}

	protected abstract double VelX(double s);

	protected abstract double VelY(double s);

	public Tuple<Double, Double> calcAcc(double s) {
		return new Tuple<>(AccX(s), AccY(s));
	}

	protected abstract double AccX(double s);

	protected abstract double AccY(double s);

	public Tuple<Double, Double> calcJerk(double s) {
		return new Tuple<>(JerkX(s), JerkY(s));
	}

	protected abstract double JerkX(double s);

	protected abstract double JerkY(double s);
}