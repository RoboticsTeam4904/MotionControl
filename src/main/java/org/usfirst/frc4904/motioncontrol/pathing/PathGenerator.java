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