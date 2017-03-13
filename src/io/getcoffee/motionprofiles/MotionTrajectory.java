package io.getcoffee.motionprofiles;


import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.TreeMap;

strictfp public class MotionTrajectory {
	public static final double robotMaxVel = MotionTrajectoryExecutor.robotMaxVel;
	public static final double robotMaxAccel = MotionTrajectoryExecutor.robotMaxAccel;
	protected SplineGenerator splineGenerator;
	protected double plantWidth;
	protected double tickTime;
	protected LinkedList<MotionTrajectorySegment> trajectorySegments;
	protected Map<Integer, MotionTrajectoryPoint> tickMap;
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
		trajectorySegments = finalizeSegments(applyBackwardConsistency(applyForwardConsistency(generateIsolatedSegments(splineGenerator.featureSegmentMap))));
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
	public LinkedList<MotionTrajectorySegment> generateIsolatedSegments(TreeMap<Double, SplineSegment> featureSegments) {
		LinkedList<MotionTrajectorySegment> trajectorySegments = new LinkedList<>();
		double maxVel = calcMaxVel(featureSegments.get(0).maxCurve);
		double maxAcc = calcMaxAcc(featureSegments.get(0).maxCurve, featureSegments.get(0).maxCurveDerivative);
		double lastFinVel = 0.0;
		for (Map.Entry<Double, SplineSegment> featureEntry : featureSegments.entrySet()) {
			MotionTrajectorySegment segment = new MotionTrajectorySegment(featureEntry.getValue().length, lastFinVel, maxVel,
				maxAcc);
			maxVel = calcMaxVel(featureEntry.getValue().maxCurve);
			maxAcc = calcMaxAcc(featureEntry.getValue().maxCurve, featureEntry.getValue().maxCurveDerivative);
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
	public Tuple<MotionTrajectoryPoint, MotionTrajectoryPoint> calcPoint(int tick, Tuple<MotionTrajectoryPoint, MotionTrajectoryPoint> lastPoints) {
		MotionTrajectoryPoint generalSetpoint = tickMap.get(tick);
		Map.Entry<Double, SplineSegment> segmentEntry = splineGenerator.featureSegmentMap.ceilingEntry(generalSetpoint.pos);
		SplinePoint splinePoint = segmentEntry.getValue().findNearestPoint(generalSetpoint.pos - segmentEntry.getKey());
		double offset = plantWidth * splineGenerator.calcCurvature(splinePoint.percentage);
		double accOffset = plantWidth * generalSetpoint.vel * splineGenerator.calcCurvatureDerivative(splinePoint.percentage);
		double leftOffset = 1 - offset;
		double rightOffset = 1 + offset;
		double rightVel = generalSetpoint.vel * rightOffset;
		double leftVel = generalSetpoint.vel * leftOffset;
		MotionTrajectoryPoint leftPoint = new MotionTrajectoryPoint(tick, lastPoints.getX().pos + leftVel * tickTime, leftVel,
				generalSetpoint.accel * leftOffset - accOffset);
		MotionTrajectoryPoint rightPoint = new MotionTrajectoryPoint(tick, lastPoints.getY().pos + rightVel * tickTime, rightVel,
			generalSetpoint.accel * rightOffset + accOffset);
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