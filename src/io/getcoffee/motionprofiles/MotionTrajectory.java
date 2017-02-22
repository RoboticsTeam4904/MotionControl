package io.getcoffee.motionprofiles;


import java.util.LinkedList;
import java.util.Map;
import io.getcoffee.motionprofiles.WheelTrajectory.Wheel;

strictfp public class MotionTrajectory {
	public static final double maxVel = 5;
	protected final SplineGenerator splineGenerator;
	protected final double plantWidth;
	protected final double tickTime, tickTotal;
	protected final WheelTrajectory leftWheel, rightWheel;
	protected final LinkedList<SplineSegment> featureSegments;
	/**
	 * Maps a tick to a left/right-specific segment, and the time that the tick occurs at relative to
	 * the beginning time of the segment.
	 */
	protected final Map<Integer, Tuple<Double, WheelTrajectorySegment>> leftWheelTickMap, rightWheelTickMap;

	/**
	 * 
	 * @param splineGenerator
	 * @param plantWidth
	 *        the width of whatever system we will be moving (in our case the robot)
	 * @param tickTime
	 *        the time that passes during a tick (in milliseconds)
	 * @param tickTotal
	 *        the total number of ticks that occur during our trajectory
	 */
	public MotionTrajectory(SplineGenerator splineGenerator, double plantWidth, double tickTime, double tickTotal) {
		this.splineGenerator = splineGenerator;
		this.plantWidth = plantWidth;
		this.tickTime = tickTime;
		this.tickTotal = tickTotal;
		// TODO: Update the threshold to reflect a real value.
		featureSegments = splineGenerator.generateFeatureSegments(0.1);
		leftWheel = new WheelTrajectory(this, featureSegments, Wheel.LEFT, tickTotal, tickTime);
		rightWheel = new WheelTrajectory(this, featureSegments, Wheel.RIGHT, tickTotal, tickTime);
		System.out.println(featureSegments.size());
		leftWheelTickMap = leftWheel.generateTickMap();
		rightWheelTickMap = rightWheel.generateTickMap();
	}
	
	public LinkedList<MotionTrajectorySegment> generateIsolatedSegments(LinkedList<SplineSegment> featureSegments) {
		LinkedList<MotionTrajectorySegment> trajectorySegments = new LinkedList<>();
		MotionTrajectorySegment lastSegment = new MotionTrajectorySegment(0);
		return trajectorySegments;
	}

	/**
	 * Calculate the right/left motion trajectory points at a given tick. For ease of calculations,
	 * a map from ticks to the trajectory segment and time at which the tick occurs is pre-generated.
	 * 
	 * It is noted that there is a possibility for a segment with a duration smaller than the time of
	 * a tick could exist. However, we deem such a segment to be insignificant and the positional error
	 * that could occur due to this disregard is assumed to be handled by positional PID.
	 * 
	 * @param tick
	 * @return
	 */
	public Tuple<MotionTrajectoryPoint, MotionTrajectoryPoint> calcPoint(int tick) {
		Tuple<Double, WheelTrajectorySegment> leftWheelTick = leftWheelTickMap.get(tick);
		Tuple<Double, WheelTrajectorySegment> rightWheelTick = rightWheelTickMap.get(tick);
		return new Tuple<>(leftWheelTick.getY().findSetPoint(leftWheelTick.getX(), tick),
			rightWheelTick.getY().findSetPoint(rightWheelTick.getX(), tick));
	}

	public double calcMaxAccel(double s) {
		return max
	}
	
	public double calcMaxSpeed(double s) {
		return calcMaxSpeedFromCurvature(splineGenerator.calcCurvature(s));
	}

	public double calcMaxSpeedFromCurvature(double curvature) {
		return maxVel / (1 + plantWidth * Math.abs(curvature) / 2);
	}

	public double calcMaxAngularVel(double s) {
		return calcTurning(s, calcMaxSpeed(s)) * plantWidth; // = theta/second * circumference/2pi = distance / second
	}

	public double calcAngularVel(double speed, double curvature) {
		return curvature * speed * plantWidth; // = theta/second * circumference/2pi = distance / second
	}

	protected double calcTurning(double s, double speed) {
		return splineGenerator.calcCurvature(s) * speed;
	}

	public Tuple<Tuple<Double, Double>, Tuple<Double, Double>> calcPerpDerivativeAndSpeed(double s) {
		Tuple<Double, Double> splineVel = splineGenerator.calcVel(s);
		Tuple<Double, Double> splineAcc = splineGenerator.calcAcc(s);
		double splineSpeed = Math.sqrt(splineVel.getX() * splineVel.getX() + splineVel.getY() * splineVel.getY());
		double splineSpeedD = (splineAcc.getX() * splineVel.getX() + splineAcc.getY() * splineVel.getY())
			/ (splineSpeed * splineSpeed); // The derivative of the speed of the spline / v
		Tuple<Double, Double> perpDerivative = new Tuple<>(
			(splineVel.getY() * splineSpeedD - splineAcc.getY()) / splineSpeed * plantWidth / 2.0,
			(splineAcc.getX() - splineVel.getX() * splineSpeedD) / splineSpeed * plantWidth / 2.0);
		return new Tuple<>(perpDerivative, splineVel);
	}
}