package org.usfirst.frc4904.motionprofiles;


import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.TreeMap;

import org.usfirst.frc4904.motionprofiles.pathing.PathGenerator;
import org.usfirst.frc4904.motionprofiles.pathing.PathPoint;
import org.usfirst.frc4904.motionprofiles.pathing.PathSegment;

strictfp public class MotionTrajectory {
	public static final double robotMaxVel = MotionTrajectoryExecutor.robotMaxVel;
	public static final double robotMaxAccel = MotionTrajectoryExecutor.robotMaxAccel;
	protected PathGenerator pathGenerator;
	protected double plantWidth;
	protected double tickTime;
	protected LinkedList<MotionTrajectorySegment> trajectorySegments;
	protected Map<Integer, MotionTrajectoryPoint> tickMap;
	protected int tickTotal;

	/**
	 * 
	 * @param pathGenerator
	 * @param plantWidth
	 *        the width of whatever system we will be moving (in our case the robot)
	 * @param tickTime
	 *        the time that passes during a tick (in milliseconds)
	 */
	public MotionTrajectory(PathGenerator pathGenerator, double plantWidth, double tickTime) {
		this.pathGenerator = pathGenerator;
		this.plantWidth = plantWidth;
		this.tickTime = tickTime / 1000;
		// TODO: Update the threshold to reflect a real value.
		trajectorySegments = finalizeSegments(
			applyBackwardConsistency(applyForwardConsistency(generateIsolatedSegments(pathGenerator.featureSegmentMap))));
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
	public LinkedList<MotionTrajectorySegment> generateIsolatedSegments(TreeMap<Double, PathSegment> featureSegments) {
		LinkedList<MotionTrajectorySegment> trajectorySegments = new LinkedList<>();
		System.out.println(featureSegments.values());
		Map.Entry<Double, PathSegment> firstEntry = featureSegments.firstEntry();
		PathSegment firstFeature = firstEntry.getValue();
		double maxVel = calcMaxVel(firstFeature.maxCurve);
		double lastFinVel = 0.0;
		for (Map.Entry<Double, PathSegment> featureEntry : featureSegments.entrySet()) {
			System.out
				.println("Max Accel: " + featureEntry.getValue().maxAcc + ", Min Accel: " + featureEntry.getValue().minAcc);
			MotionTrajectorySegment segment = new MotionTrajectorySegment(featureEntry.getValue().length, lastFinVel, maxVel,
				featureEntry.getValue().maxAcc, featureEntry.getValue().minAcc);
			maxVel = calcMaxVel(featureEntry.getValue().maxCurve);
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
		for (int i = trajectorySegments.size() - 1; i > -1; i--) {
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
			timePassed += segment.duration;
			distanceTraveled += segment.length;
		}
		System.out.println(trajectorySegments);
		System.out.println(timePassed);
		return trajectorySegments;
	}

	/**
	 * Generate map from ticks to set-points
	 * 
	 * @param trajectorySegments
	 * @return
	 */
	public Map<Integer, MotionTrajectoryPoint> generateFullTickMap(LinkedList<MotionTrajectorySegment> trajectorySegments) {
		HashMap<Integer, MotionTrajectoryPoint> map = new HashMap<>();
		double timeOverSegment = 0.0;
		double distanceTraveled = 0.0;
		int tickCount = 0;
		for (int segmentIndex = 0; segmentIndex < trajectorySegments.size(); segmentIndex++) {
			MotionTrajectorySegment segment = trajectorySegments.get(segmentIndex);
			for (; timeOverSegment < segment.duration; timeOverSegment += tickTime) {
				MotionTrajectoryPoint point = segment.calcOffsetSetpoint(timeOverSegment, tickCount, distanceTraveled);
				map.put(tickCount, point);
				tickCount++;
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
	public Tuple<MotionTrajectoryPoint, MotionTrajectoryPoint> calcPoint(int tick,
		Tuple<MotionTrajectoryPoint, MotionTrajectoryPoint> lastPoints) {
		MotionTrajectoryPoint generalSetpoint = tickMap.get(tick);
		Map.Entry<Double, PathSegment> segmentEntry = pathGenerator.featureSegmentMap.floorEntry(generalSetpoint.pos);
		PathPoint pathPoint = segmentEntry.getValue().findNearestPoint(generalSetpoint.pos - segmentEntry.getKey());
		double offset = plantWidth * pathGenerator.calcCurvature(pathPoint.percentage);
		double accOffset = plantWidth * generalSetpoint.vel * pathGenerator.calcCurvatureDerivative(pathPoint.percentage);
		double leftOffset = 1 - offset;
		double rightOffset = 1 + offset;
		double rightVel = generalSetpoint.vel * rightOffset;
		double leftVel = generalSetpoint.vel * leftOffset;
		MotionTrajectoryPoint leftPoint = new MotionTrajectoryPoint(tick, lastPoints.getX().pos + leftVel * tickTime, leftVel,
			(leftVel - lastPoints.getX().vel) / tickTime);
		MotionTrajectoryPoint rightPoint = new MotionTrajectoryPoint(tick, lastPoints.getY().pos + rightVel * tickTime,
			rightVel, (rightVel - lastPoints.getY().vel) / tickTime);
		return new Tuple<>(leftPoint, rightPoint);
	}

	public double calcMaxAcc(double curvature, double curveDerivative, double maxVel,
		double maxSplineVel) {
		double max1 = (robotMaxAccel + Math.signum(curvature) * (plantWidth * maxVel * maxVel * curveDerivative / maxSplineVel)
			/ calcDivisor(curvature)); // Or flip signs? equivalent? Also, make better by finding min val of maxAccel across the segment by making all of these (including V of robot?) functions of s and solving or probably just check at various values of s
		double max2 = (robotMaxAccel - Math.signum(curvature) * (plantWidth * maxVel * maxVel * curveDerivative / maxSplineVel)
			/ calcDivisor(-curvature));
		if (max1 > 2) {
			return max1;
		} else {
			return max2;
		}
	}

	public double calcMinAcc(double curvature, double curveDerivative, double maxVel, double maxSplineVel) {
		double max1 = (robotMaxAccel + Math.signum(curvature) * (plantWidth * maxVel * maxVel * curveDerivative / maxSplineVel)
			/ calcDivisor(curvature)); // Or flip signs? equivalent? Also, make better by finding min val of maxAccel across the segment by making all of these (including V of robot?) functions of s and solving or probably just check at various values of s
		double max2 = (robotMaxAccel - Math.signum(curvature) * (plantWidth * maxVel * maxVel * curveDerivative / maxSplineVel)
			/ calcDivisor(-curvature));
		if (max1 < 2) {
			return max1;
		} else {
			return max2;
		}
	}

	public double calcMaxVel(double curvature) {
		return robotMaxVel / calcDivisor(curvature);
	}

	private double calcDivisor(double curvature) {
		return 1 - Math.signum(curvature) * (plantWidth * Math.abs(curvature));
	}

	public int getTickTotal() {
		return tickTotal;
	}
}