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
		trajectorySegments = finalizeSegments(
			applyBackwardConsistency(applyForwardConsistency(generateIsolatedSegments(splineGenerator.featureSegmentMap))));
		System.out.println(trajectorySegments.getFirst());
		tickMap = generateFullTickMap(trajectorySegments);
		System.out.println("Tick Map" + tickMap);
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
		System.out.println(featureSegments.values());
		Map.Entry<Double, SplineSegment> firstEntry = featureSegments.firstEntry();
		SplineSegment firstFeature = firstEntry.getValue();
		double maxVel = calcMaxVel(firstFeature.maxCurve);
		double maxAcc = calcMaxAcc(firstFeature.maxCurve, firstFeature.maxCurveDerivative, maxVel, firstFeature.maxSpeed);
		double lastFinVel = 0.0;
		for (Map.Entry<Double, SplineSegment> featureEntry : featureSegments.entrySet()) {
			MotionTrajectorySegment segment = new MotionTrajectorySegment(featureEntry.getValue().length, lastFinVel, maxVel,
				maxAcc);
			maxVel = calcMaxVel(featureEntry.getValue().maxCurve);
			maxAcc = calcMaxAcc(featureEntry.getValue().maxCurve, featureEntry.getValue().maxCurveDerivative, maxVel,
				featureEntry.getValue().maxSpeed);
			System.out.println(maxAcc);
			segment.finVel = Math.min(segment.maxVel, maxVel);
			trajectorySegments.add(segment);
			lastFinVel = segment.finVel;
		}
		System.out.println("Isolated" + trajectorySegments);
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
		System.out.println(trajectorySegments);
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
		System.out.println(trajectorySegments);
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
		System.out.println(trajectorySegments);
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
			System.out.println(tickCount);
			System.out.println(timeOverSegment);

			MotionTrajectorySegment segment = trajectorySegments.get(tickCount);
			for (; timeOverSegment < segment.duration; timeOverSegment += tickTime) {
				MotionTrajectoryPoint point = segment.calcOffsetSetpoint(timeOverSegment, tickCount, distanceTraveled);
				System.out.println(point);
				map.put(tickCount, point);
			}
			timeOverSegment -= segment.duration;
			distanceTraveled += segment.length;
		}
		this.tickTotal = tickCount;
		System.out.println(map);
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
	public Tuple<MotionTrajectoryPoint, MotionTrajectoryPoint> calcPoint(int tick,
		Tuple<MotionTrajectoryPoint, MotionTrajectoryPoint> lastPoints) {
		MotionTrajectoryPoint generalSetpoint = tickMap.get(tick);
		System.out.println(generalSetpoint);
		Map.Entry<Double, SplineSegment> segmentEntry = splineGenerator.featureSegmentMap.floorEntry(generalSetpoint.pos);
		SplinePoint splinePoint = segmentEntry.getValue().findNearestPoint(generalSetpoint.pos - segmentEntry.getKey());
		double offset = plantWidth * splineGenerator.calcCurvature(splinePoint.percentage);
		double accOffset = plantWidth * generalSetpoint.vel * splineGenerator.calcCurvatureDerivative(splinePoint.percentage);
		double leftOffset = 1 - offset;
		double rightOffset = 1 + offset;
		double rightVel = generalSetpoint.vel * rightOffset;
		double leftVel = generalSetpoint.vel * leftOffset;
		MotionTrajectoryPoint leftPoint = new MotionTrajectoryPoint(tick, lastPoints.getX().pos + leftVel * tickTime, leftVel,
			generalSetpoint.accel * leftOffset - accOffset);
		MotionTrajectoryPoint rightPoint = new MotionTrajectoryPoint(tick, lastPoints.getY().pos + rightVel * tickTime,
			rightVel,
			generalSetpoint.accel * rightOffset + accOffset);
		return new Tuple<>(rightPoint, leftPoint);
	}

	public double calcMaxAcc(double curvature, double curveDerivative, double maxVel, double maxSplineVel) {
		return (robotMaxAccel + (plantWidth * maxVel * maxVel * curveDerivative / maxSplineVel) / calcDivisor(curvature)) * Math.signum(curvature); // Or flip signs? equivalent? Also, make better by finding min val of maxAccel across the segment by making all of these (including V of robot?) functions of s and solving or probably just check at various values of s
	}

	public double calcMaxVel(double curvature) {
		return robotMaxVel / calcDivisor(curvature);
	}

	private double calcDivisor(double curvature) {
		return 1 - (plantWidth * Math.abs(curvature)) * Math.signum(curvature);
	}

	public int getTickTotal() {
		return tickTotal;
	}
}