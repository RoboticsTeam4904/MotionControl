package io.getcoffee.motionprofiles;


import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

strictfp public class MotionTrajectory {
	public static final double robotMaxVel = MotionTrajectoryExecutor.robotMaxVel;
	public static final double robotMaxAccel = MotionTrajectoryExecutor.robotMaxAccel;
	protected final SplineGenerator splineGenerator;
	protected final double plantWidth;
	protected final double tickTime;
	protected final LinkedList<SplineSegment> featureSegments;
	protected final LinkedList<MotionTrajectorySegment> trajectorySegments;
	protected final Map<Integer, MotionTrajectoryPoint> tickMap;
	protected int tickTotal;

	/**
	 * 
	 * @param splineGenerator
	 * @param plantWidth
	 *        the width of whatever system we will be moving (in our case the robot)
	 * @param tickTime
	 *        the time that passes during a tick (in milliseconds)
	 */
	public MotionTrajectory(SplineGenerator splineGenerator, double plantWidth, double tickTime) {
		this.splineGenerator = splineGenerator;
		this.plantWidth = plantWidth / 2.0;
		this.tickTime = tickTime;
		// TODO: Update the threshold to reflect a real value.
		featureSegments = splineGenerator.generateFeatureSegments(0.1);
		trajectorySegments = finalizeSegments(
			applyBackwardConsistency(applyForwardConsistency(generateIsolatedSegments(featureSegments))));
		tickMap = generateFullTickMap(trajectorySegments);
	}

	/**
	 * Generates a set of ideal segments which do not yet account for true forwards
	 * or backwards consistency between the previous or future segments given feature
	 * segments of the spline.
	 *
	 * This calculates
	 * the fastest possible velocity and acceleration for segment P<sub>i</sub>, then
	 * moves onto the segment P<sub>i + 1</sub> and calculates the fastest possible velocity
	 * and acceleration. Then to ensure that the transition between the two segments is
	 * physically possible, sets the exit velocity of P<sub>i</sub> to the lowest
	 * value between the P<sub>i</sub> and P<sub>i + 1</sub>'s maximum velocities.
	 *
	 * @param featureSegments
	 * @return
	 */
	public LinkedList<MotionTrajectorySegment> generateIsolatedSegments(LinkedList<SplineSegment> featureSegments) {
		LinkedList<MotionTrajectorySegment> trajectorySegments = new LinkedList<>();
		double maxVel = calcMaxVel(featureSegments.get(0).maxCurve);
		double maxAcc = calcMaxAcc(featureSegments.get(0).maxCurve, featureSegments.get(0).maxCurveDerivative);
		double lastFinVel = 0.0;
		for (SplineSegment featureSegment : featureSegments) {
			MotionTrajectorySegment segment = new MotionTrajectorySegment(featureSegment.length, lastFinVel, maxVel,
				maxAcc);
			maxVel = calcMaxVel(featureSegment.maxCurve);
			maxAcc = calcMaxAcc(featureSegment.maxCurve, featureSegment.maxCurveDerivative);
			segment.finVel = Math.min(segment.maxVel, maxVel);
			trajectorySegments.add(segment);
			lastFinVel = segment.finVel;
		}
		return trajectorySegments;
	}

	/**
	 * Adjusts the provided trajectory segments to ensure forward consistency between them.
	 * This is done by lowering the final velocity to fit acceleration constraints, making
	 * the transition between segments possible.
	 * 
	 * @see {@link MotionTrajectory#applyBackwardConsistency}
	 * @see {@link MotionTrajectory#finalizeSegments}
	 * 
	 * @param trajectorySegments
	 * @return ordered right/left trajectory segments that are forward consistent.
	 */
	public LinkedList<MotionTrajectorySegment> applyForwardConsistency(LinkedList<MotionTrajectorySegment> trajectorySegments) {
		double lastFinVel = 0.0;
		for (MotionTrajectorySegment segment : trajectorySegments) {
			segment.initVel = lastFinVel;
			segment.finVel = Math.min(segment.calcReachableEndVel(), segment.finVel);
			lastFinVel = segment.finVel;
		}
		return trajectorySegments;
	}

	/**
	 * Adjusts the provided trajectory segments to ensure backward consistency between them.
	 * This is done by lowering the initial velocity to fit deceleration constraints, making
	 * the transition between segments possible.
	 *
	 * @see {@link MotionTrajectory#applyForwardConsistency}
	 * @see {@link MotionTrajectory#finalizeSegments}
	 *
	 * @param trajectorySegments
	 *        ordered segments that are forward consistent
	 * @return ordered right/left trajectory segments that are forward and backward consistent.
	 */
	public LinkedList<MotionTrajectorySegment> applyBackwardConsistency(
		LinkedList<MotionTrajectorySegment> trajectorySegments) {
		double lastInitVel = 0.0;
		for (int i = trajectorySegments.size() - 1; i > 0; i--) {
			MotionTrajectorySegment trajectorySegment = trajectorySegments.get(i);
			trajectorySegment.finVel = lastInitVel;
			trajectorySegment.initVel = Math.min(trajectorySegment.calcReachableStartVel(), trajectorySegment.initVel);
			lastInitVel = trajectorySegment.initVel;
		}
		return trajectorySegments;
	}

	/**
	 * Attach an absolute context and finalize the path of each of the segments.
	 * 
	 * @param trajectorySegments
	 *        ordered segments that are forward and backward consistent.
	 * @return finalized ordered right/left trajectory segments with absolute context attached.
	 */
	public LinkedList<MotionTrajectorySegment> finalizeSegments(LinkedList<MotionTrajectorySegment> trajectorySegments) {
		double timePassed = 0;
		double distanceTraveled = 0;
		for (MotionTrajectorySegment segment : trajectorySegments) {
			segment.dividePath();
			segment.context = new AbsoluteSegmentContext(timePassed, distanceTraveled);
			timePassed += segment.duration;
			distanceTraveled += segment.length;
		}
		return trajectorySegments;
	}

	/**
	 * Finalize segments and generate map from ticks to set-points
	 * 
	 * @param trajectorySegments
	 * @return
	 */
	public Map<Integer, MotionTrajectoryPoint> generateFullTickMap(LinkedList<MotionTrajectorySegment> trajectorySegments) {
		HashMap<Integer, MotionTrajectoryPoint> map = new HashMap<>();
		double timeOverSegment = 0.0;
		double distanceTraveled = 0.0;
		int tickCount = 0;
		for (; tickCount < trajectorySegments.size(); tickCount++) {
			MotionTrajectorySegment segment = trajectorySegments.get(tickCount);
			segment.dividePath();
			for (; timeOverSegment < segment.duration; timeOverSegment += tickTime) {
				map.put(tickCount, segment.calcOffsetSetpoint(timeOverSegment, tickCount, distanceTraveled));
			}
			timeOverSegment -= segment.duration;
			distanceTraveled += segment.length;
		}
		this.tickTotal = tickCount;
		return map;
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
		MotionTrajectoryPoint generalSetpoint = tickMap.get(tick);
		double s = splineGenerator.calcSFromPosAndSegment(generalSetpoint.pos, generalSetpoint.segment); // relative position (currently absolute). Make intermediate class
		double offSet = plantWidth * splineGenerator.calcCurvature(s);
		double accOffSet = plantWidth * generalSetpoint.vel * splineGenerator.calcCurvatureDerivative(s);
		double rightOffSet = 1 + offSet;
		double leftOffSet = 1 - offSet;
		double rightVel = generalSetpoint.vel * rightOffSet;
		double leftVel = generalSetpoint.vel * leftOffSet;
		MotionTrajectoryPoint rightPoint = new MotionTrajectoryPoint(tick, lastRightPos + rightVel * tickTime, rightVel,
			generalSetpoint.accel * rightOffSet + accOffSet);
		MotionTrajectoryPoint leftPoint = new MotionTrajectoryPoint(tick, lastLeftPos + leftVel * tickTime, leftVel,
			generalSetpoint.accel * leftOffSet - accOffSet);
		return new Tuple<>(rightPoint, leftPoint);
	}

	public double calcMaxAcc(double curvature, double curveDerivative) {
		return (robotMaxAccel - plantWidth * calcMaxVel(curvature) * curveDerivative) / calcDivisor(curvature);
	}

	public double calcMaxVel(double curvature) {
		return robotMaxVel / calcDivisor(curvature);
	}

	private double calcDivisor(double curvature) {
		return 1 + plantWidth * Math.abs(curvature);
	}

	public double calcMaxSpeed(double s) {
		return calcMaxSpeedFromCurvature(splineGenerator.calcCurvature(s));
	}

	public double calcMaxSpeedFromCurvature(double curvature) {
		return robotMaxVel / (1 + plantWidth * Math.abs(curvature) / 2);
	}

	public double calcMaxAngularVel(double s) {
		return calcMaxAngularVelFromCurvature(splineGenerator.calcCurvature(s));
	}

	public double calcMaxAngularVelFromCurvature(double curvature) {
		return calcAngularVel(calcMaxSpeedFromCurvature(curvature), curvature);
	}

	public double calcAngularVel(double speed, double curvature) {
		return curvature * speed * plantWidth; // = theta/meter * meter/second * circumference/2pi = distance / second
	}

	public int getTickTotal() {
		return tickTotal;
	}
}