package io.getcoffee.motionprofiles;


import java.util.LinkedList;
import java.util.Map;
import io.getcoffee.motionprofiles.WheelTrajectory.Wheel;

strictfp public class MotionTrajectory {
	public static final double robotMaxVel = MotionTrajectoryExecutor.robotMaxVel;
	public static final double robotMaxAccel = MotionTrajectoryExecutor.robotMaxAccel;
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
		this.plantWidth = plantWidth/2.0;
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
	
	public LinkedList<MotionTrajectorySegment> generateSegmentsAndForwardPass(LinkedList<SplineSegment> featureSegments) {
		LinkedList<MotionTrajectorySegment> trajectorySegments = new LinkedList<>();
		SplineSegment featureSegment = featureSegments.get(0); 
		Tuple<Double,Double> velAndAccel = calcMaxVelAndAccFromCurvatureAndDerivative(featureSegment.maxCurvature, featureSegment.maxDCurvature);
		MotionTrajectorySegment lastSegment = new MotionTrajectorySegment(0.0); // Initialize velocity at 0
		for (int i = 0; i < featureSegments.size(); i++) {
			featureSegment = featureSegments.get(i);
			lastSegment = new MotionTrajectorySegment(featureSegment.length, lastSegment.finVel, velAndAccel.getX(), velAndAccel.getY());
			velAndAccel = calcMaxVelAndAccFromCurvatureAndDerivative(featureSegment.maxCurvature, featureSegment.maxDCurvature);
			lastSegment.finVel = Math.min(lastSegment.calcReachableEndVel(), Math.min(lastSegment.maxVel, velAndAccel.getX()));
			trajectorySegments.add(lastSegment);
		}
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

	/**
	 * Super constraining right now. Uses maxVel, maxCurvature and maxCurvatureD.
 	 * It would ideally find the minAccel given the maxCurvature and maxCurvatureD at every point
 	 * As well as vel at that point as a function of maxVel and accel itself (simultaneous calculation possible?)
	 * @param curvature
	 * @param dCurvature
	 * @return
	 */
	public Tuple<Double,Double> calcMaxAccelFromCurvatureAndDerivative(double curvature, double dCurvature) {
		double divisor = (1+plantWidth*Math.abs(curvature));
		double maxVel = robotMaxVel/divisor;
		return Tuple(maxVel, (robotMaxAccel-plantWidth*maxVel*dCurvature)/divisor);
	}
	
	public double calcMaxSpeed(double s) {
		return calcMaxSpeedFromCurvature(splineGenerator.calcCurvature(s));
	}

	public double calcMaxSpeedFromCurvature(double curvature) {
		return maxVel / (1 + plantWidth * Math.abs(curvature) / 2);
	}

	public double calcMaxAngularVel(double s) {
		return calcMaxAngularVelFromCurvature(splineGenerator.calcCurvature(s));
	}
	public double calcMaxAngularVelFromCurvature(double curvature) {
		return  calcAngularVel(calcMaxSpeedFromCurvature(curvature), curvature);
	}

	public double calcAngularVel(double speed, double curvature) {
		return curvature * speed * plantWidth; // = theta/meter * meter/second * circumference/2pi = distance / second
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