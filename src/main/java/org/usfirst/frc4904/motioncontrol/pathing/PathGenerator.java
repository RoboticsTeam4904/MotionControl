package org.usfirst.frc4904.motioncontrol.pathing;

import java.util.TreeMap;
import org.usfirst.frc4904.motioncontrol.MotionTrajectoryExecutor;
import org.usfirst.frc4904.motioncontrol.Tuple;


strictfp public abstract class PathGenerator {
	public static final double INTEGRATION_GRANULARITY = 20;
	public static final double CURVATURE_THRESHOLD = 0.3;
	public static double robotMaxAccel = MotionTrajectoryExecutor.robotMaxAccel;
	public static double robotMinAccel = -MotionTrajectoryExecutor.robotMaxAccel;
	public static double robotMaxVel = MotionTrajectoryExecutor.robotMaxVel;
	public static double plantWidth = MotionTrajectoryExecutor.plantWidth;
	public TreeMap<Double, PathSegment> featureSegmentMap = new TreeMap<>();
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
	public void segment(double curveDerivativeThreshold, double granularity) {
		double initCurve = calcCurvature(0.0);
		double maxVel = robotMaxVel;
		double minAcc = robotMinAccel;
		double maxAcc = robotMaxAccel;
		double absoluteArcSum = 0.0;
		double arcSum = 0.0;
		TreeMap<Double, Double> localLengthMap = new TreeMap<>();
		double lastCurve = initCurve;
		for (int i = 0; i <= granularity; i += 1) {
			double percentage = i/granularity; // To avoid floating point errors for the last iteration, we iterate with an int and then divide by granularity
			localLengthMap.put(arcSum, percentage);
			double instantSpeed = calcSpeed(percentage);
			arcSum += instantSpeed / granularity;
			double instantCurve = calcCurvature(percentage);
			// double instantCurveDerivative = calcCurvatureDerivative(percentage);
			double instantCurveDerivative = (instantCurve - lastCurve) * granularity;
			double rightModifier = 1 + plantWidth * instantCurve;
			double leftModifier = 1 - plantWidth * instantCurve;
			double instantMaxVel = robotMaxVel / Math.max(Math.abs(leftModifier), Math.abs(rightModifier));; // equivalent to without absolute values, but readably accounts for negative maximal velocity

			// double minVelSqrd = 0;
			double maxVelSqrd = instantMaxVel * instantMaxVel;
			System.out.println(percentage + ", " + instantCurveDerivative + ", " + calcCurvatureDerivative(percentage) + ", " + instantCurve); //+ instantSpeed + ", " + instantCurveDerivative / instantSpeed + ", " + maxVelSqrd);

			double maxAccRightMaxVel = (robotMaxAccel - plantWidth * maxVelSqrd * instantCurveDerivative / instantSpeed)
				/ rightModifier;
			double maxAccRightMinVel = robotMaxAccel / rightModifier;
			double minAccRightMaxVel = (robotMinAccel - plantWidth * maxVelSqrd * instantCurveDerivative / instantSpeed)
				/ rightModifier;
			double minAccRightMinVel = robotMinAccel / rightModifier;
			double maxAccRight, minAccRight;
			if (rightModifier > 0) {
				maxAccRight = Math.min(maxAccRightMaxVel, maxAccRightMinVel);
				minAccRight = Math.max(minAccRightMaxVel, minAccRightMinVel);
			} else {
				maxAccRight = Math.min(minAccRightMaxVel, minAccRightMinVel);
				minAccRight = Math.max(maxAccRightMaxVel, maxAccRightMinVel);
			}
			
			double maxAccLeftMaxVel = (robotMaxAccel + plantWidth * maxVelSqrd * instantCurveDerivative / instantSpeed)
				/ leftModifier;
			double maxAccLeftMinVel = robotMaxAccel / leftModifier;
			double minAccLeftMaxVel = (robotMinAccel + plantWidth * maxVelSqrd * instantCurveDerivative / instantSpeed)
				/ leftModifier;
			double minAccLeftMinVel = robotMinAccel / leftModifier;
			double maxAccLeft, minAccLeft;
			if (leftModifier > 0) {
				maxAccLeft = Math.min(maxAccLeftMaxVel, maxAccLeftMinVel);
				minAccLeft = Math.max(minAccLeftMaxVel, minAccLeftMinVel);
			} else {
				maxAccLeft = Math.min(minAccLeftMaxVel, minAccLeftMinVel);
				minAccLeft = Math.max(maxAccLeftMaxVel, maxAccLeftMinVel);
			}

			double instantMaxAcc = Math.min(maxAccLeft, maxAccRight);
			double instantMinAcc = Math.max(minAccLeft, minAccRight);

			maxVel = Math.min(maxVel, instantMaxVel);
			minAcc = Math.max(minAcc, instantMinAcc);
			maxAcc = Math.min(maxAcc, instantMaxAcc);
			if (Math.abs(initCurve - instantCurve) > curveDerivativeThreshold) {
				featureSegmentMap.put(absoluteArcSum, new PathSegment(maxVel, minAcc, maxAcc, arcSum, localLengthMap));
				maxVel = robotMaxVel;
				minAcc = robotMinAccel;
				maxAcc = robotMaxAccel;
				absoluteArcSum += arcSum;
				arcSum = 0.0;
				localLengthMap = new TreeMap<>();
				initCurve = instantCurve;
			}
			lastCurve = instantCurve;
		}
		localLengthMap.put(arcSum, 1.);
		featureSegmentMap.put(absoluteArcSum, new PathSegment(maxVel, minAcc, maxAcc, arcSum, localLengthMap)); //an issue
		absoluteLength = arcSum;
	}

	protected void segment(double threshold) {
		segment(threshold, INTEGRATION_GRANULARITY);
	}

	protected void segment() {
		segment(CURVATURE_THRESHOLD, INTEGRATION_GRANULARITY);
	}

	/**
	 * Calculate a map from arclength (along segment) to s at a<=s<=b
	 * 
	 * @param a
	 * @param b
	 * @param granularity
	 * @return
	 */
	public TreeMap<Double, Double> calcFeatureLengthMap(double a, double b, double granularity) {
		TreeMap<Double, Double> map = new TreeMap<Double, Double>();
		double length = 0;
		for (double i = a; i <= b; i += 1 / granularity) {
			length += calcSpeed(i);
			map.put(length, i);
		}
		return map;
	}

	/**
	 * Calculate a map from arclength (along segment) to s at a<=s<=b
	 * 
	 * @param a
	 * @param b
	 * @return
	 */
	public TreeMap<Double, Double> calcFeatureLengthMap(double a, double b) {
		return calcFeatureLengthMap(a, b, INTEGRATION_GRANULARITY);
	}

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
		// TODO: if speed == 0: use jerk and acc instead of acc and vel. if acc == 0,
		// just return 0 or go deeper or something
		// System.out.println(speed);
		// System.out.println("s:" + s + ", " + speed + ", " + (vel.getX() * acc.getY() - vel.getY() * acc.getX()) / (speed * speed * speed));
		return (vel.getX() * acc.getY() - vel.getY() * acc.getX()) / (speed * speed * speed);
	}

	/**
	 * Equation for the derivative of curvature at a percentage of the
	 * arc-length.
	 *
	 * @see <a href=
	 *      "https://www.wolframalpha.com/input/?i=(x%27(t)+*+y%27%27(t)+-+y%27(t)+*+x%27%27(t))%2F(x%27(t)%5E2+%2B+y%27(t)%5E2)%5E(3%2F2)">Wolfram-Alpha
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
	 * @param s the position along the spline from [0-1]
	 * @return the 'speed' (note that this is not true physical speed but
	 *         merely the magnitude of the spline velocity)
	 */
	public double calcSpeed(double s) {
		Tuple<Double, Double> vel = calcVel(s);
		return Math.sqrt(vel.getX() * vel.getX() + vel.getY() * vel.getY());
	}

	/**
	 * @param s the position along the spline from [0-1]
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