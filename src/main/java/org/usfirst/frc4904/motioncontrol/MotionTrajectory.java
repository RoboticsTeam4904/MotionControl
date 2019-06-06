package org.usfirst.frc4904.motioncontrol;


import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.TreeMap;
import org.usfirst.frc4904.motioncontrol.pathing.PathGenerator;
import org.usfirst.frc4904.motioncontrol.pathing.PathSegment;

strictfp public class MotionTrajectory {
	public static final double INTEGRATION_GRANULARITY = 20;
	public static final double CURVATURE_THRESHOLD = 0.3;
	public static final double robotMaxVel = MotionTrajectoryExecutor.robotMaxVel;
	public static final double robotMaxAccel = MotionTrajectoryExecutor.robotMaxAccel;
	public static final double robotMinAccel = -MotionTrajectoryExecutor.robotMaxAccel;
	protected PathGenerator pathGenerator;
	protected double plantWidth;
	protected double tickTime;
	protected LinkedList<MotionTrajectorySegment> trajectorySegments;
	protected TreeMap<Double, PathSegment> featureSegments;
	protected Map<Integer, MotionTrajectoryPoint> tickMap;
	protected int tickTotal;

	/**
	 * 
	 * @param pathGenerator
	 * @param plantWidth
	 *                      the width of whatever system we will be moving (in our case the robot)
	 * @param tickTime
	 *                      the time that passes during a tick (in milliseconds)
	 */
	public MotionTrajectory(PathGenerator pathGenerator, double plantWidth, double tickTime) {
		this.pathGenerator = pathGenerator;
		this.plantWidth = plantWidth;
		this.tickTime = tickTime / 1000;
		// TODO: Update the threshold to reflect a real value.
		this.featureSegments = constrainAndSegment(pathGenerator);
		trajectorySegments = finalizeSegments(
			applyBackwardConsistency(applyForwardConsistency(generateIsolatedSegments(featureSegments))));
		tickMap = generateFullTickMap(trajectorySegments);
		System.out.println("Tick Map: " + tickMap);
	}

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
	public TreeMap<Double, PathSegment> constrainAndSegment(PathGenerator pathGenerator, double curveDerivativeThreshold,
		double granularity) {
		TreeMap<Double, PathSegment> featureSegmentMap = new TreeMap<>();
		double initCurve = pathGenerator.calcCurvature(0.0);
		double maxVel = robotMaxVel;
		double minAcc = robotMinAccel;
		double maxAcc = robotMaxAccel;
		double absoluteArcSum = 0.0;
		double arcSum = 0.0;
		TreeMap<Double, Double> localLengthMap = new TreeMap<>();
		double lastCurve = initCurve;
		for (int i = 0; i <= granularity; i += 1) {
			double percentage = i / granularity; // To avoid floating point errors for the last iteration, we iterate with an int and then divide by granularity
			localLengthMap.put(arcSum, percentage);
			double instantSpeed = pathGenerator.calcSpeed(percentage);
			arcSum += instantSpeed / granularity;
			double instantCurve = pathGenerator.calcCurvature(percentage);
			double instantCurveDerivative = (instantCurve - lastCurve) * granularity; // calcCurvatureDerivative(percentage);
			System.out.println(percentage + ", " + instantCurveDerivative + ", "
				+ pathGenerator.calcCurvatureDerivative(percentage) + ", " + instantCurve);
			double rightModifier = 1 + plantWidth * instantCurve;
			double leftModifier = 1 - plantWidth * instantCurve;
			double instantMaxVel = robotMaxVel / Math.max(Math.abs(leftModifier), Math.abs(rightModifier));
			; // equivalent to without absolute values, but readably accounts for negative maximal velocity
				// double minVelSqrd = 0;
			double maxVelSqrd = instantMaxVel * instantMaxVel;
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
			System.out.println("Max Accel: Left - " + maxAccLeft + "; Right - " + maxAccRight);
			double instantMaxAcc = Math.min(maxAccLeft, maxAccRight);
			double instantMinAcc = Math.max(minAccLeft, minAccRight);
			maxVel = Math.min(maxVel, instantMaxVel);
			minAcc = Math.max(minAcc, instantMinAcc);
			System.out.println("Instant Max Acc at " + i + ": " + instantMaxAcc);
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
		featureSegmentMap.put(absoluteArcSum, new PathSegment(maxVel, minAcc, maxAcc, arcSum, localLengthMap)); // an issue
		return featureSegmentMap;
	}

	protected TreeMap<Double, PathSegment> constrainAndSegment(PathGenerator pathGenerator, double threshold) {
		return constrainAndSegment(pathGenerator, threshold, INTEGRATION_GRANULARITY);
	}

	protected TreeMap<Double, PathSegment> constrainAndSegment(PathGenerator pathGenerator) {
		return constrainAndSegment(pathGenerator, CURVATURE_THRESHOLD, INTEGRATION_GRANULARITY);
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
		// System.out.println(featureSegments.values());
		MotionTrajectorySegment lastSegment = new MotionTrajectorySegment(0.0); // set initial velocity to 0
		for (Map.Entry<Double, PathSegment> featureEntry : featureSegments.entrySet()) {
			System.out
				.println("Max Accel: " + featureEntry.getValue().maxAcc + ",\tMin Accel: " + featureEntry.getValue().minAcc);
			double maxVel = featureEntry.getValue().maxVel;
			double init_vel = Math.min(lastSegment.maxVel, maxVel); // maximal initial velocity is constrained by the maximum velocity of both segments
			MotionTrajectorySegment segment = new MotionTrajectorySegment(featureEntry.getValue().length, init_vel, maxVel,
				featureEntry.getValue().maxAcc, featureEntry.getValue().minAcc);
			lastSegment.finVel = init_vel;
			trajectorySegments.add(segment); // TODO: ensure is mutable after adding to map, since finVel will be set
			lastSegment = segment;
		}
		lastSegment.finVel = 0.0; // set final velocity to 0
		// System.out.println("Isolated:\t\t" + trajectorySegments);
		return trajectorySegments;
	}

	// TODO: Combine the following functions
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
		// System.out.println("Forward consistency: \t" + trajectorySegments);
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
	 *                           ordered segments that are forward consistent
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
		// System.out.println("Backward consistency: \t" + trajectorySegments);
		System.out.println("Finalized: \t\t" + trajectorySegments);
		return trajectorySegments;
	}

	/**
	 * Profile the velocities across each of the segments.
	 * 
	 * @param trajectorySegments
	 *                           ordered segments that are forward and backward consistent.
	 * @return finalized ordered right/left trajectory segments with absolute context attached.
	 */
	public LinkedList<MotionTrajectorySegment> finalizeSegments(LinkedList<MotionTrajectorySegment> trajectorySegments) {
		System.out.println("Finalized: \t\t" + trajectorySegments);
		double timePassed = 0;
		double distanceTraveled = 0;
		for (MotionTrajectorySegment segment : trajectorySegments) {
			System.out.println("Looping ---------------");
			segment.dividePath();
			timePassed += segment.duration;
			distanceTraveled += segment.length;
		}
		System.out.println("Finalized: \t\t" + trajectorySegments);
		System.out.println("T=" + timePassed);
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
			// System.out.println(distanceTraveled + ", " + timeOverSegment + ", " + tickCount);
		}
		map.put(tickCount, new MotionTrajectoryPoint(tickCount, distanceTraveled, 0.0, 0.0)); // Add final point (0 vel, 0 acc)
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
		Map.Entry<Double, PathSegment> segmentEntry = featureSegments.floorEntry(generalSetpoint.pos);
		// System.out.println(tick + ", " + generalSetpoint.pos + ", " + segmentEntry);
		double percentage = segmentEntry.getValue().extrapolatePercentage(generalSetpoint.pos - segmentEntry.getKey());
		// System.out.println("_s: " + percentage);
		double rightVel = generalSetpoint.vel * (1 + plantWidth * pathGenerator.calcCurvature(percentage));
		double leftVel = generalSetpoint.vel * (1 - plantWidth * pathGenerator.calcCurvature(percentage));
		MotionTrajectoryPoint leftPoint = new MotionTrajectoryPoint(tick, lastPoints.getX().pos + leftVel * tickTime, leftVel,
			(leftVel - lastPoints.getX().vel) / tickTime);
		MotionTrajectoryPoint rightPoint = new MotionTrajectoryPoint(tick, lastPoints.getY().pos + rightVel * tickTime,
			rightVel, (rightVel - lastPoints.getY().vel) / tickTime);
		return new Tuple<>(leftPoint, rightPoint);
	}
	// public double calcMaxAcc(double curvature, double curveDerivative, double maxVel,
	// double maxSplineVel) {
	// double max1 = (robotMaxAccel + Math.signum(curvature) * (plantWidth * maxVel * maxVel * curveDerivative / maxSplineVel)
	// / calcDivisor(curvature)); // Or flip signs? equivalent? Also, make better by finding min val of maxAccel across the segment by making all of these (including V of robot?) functions of s and solving or probably just check at various values of s
	// double max2 = (robotMaxAccel - Math.signum(curvature) * (plantWidth * maxVel * maxVel * curveDerivative / maxSplineVel)
	// / calcDivisor(-curvature));
	// if (max1 > 2) {
	// return max1;
	// } else {
	// return max2;
	// }
	// }

	// public double calcMinAcc(double curvature, double curveDerivative, double maxVel, double maxSplineVel) {
	// double max1 = (robotMaxAccel + Math.signum(curvature) * (plantWidth * maxVel * maxVel * curveDerivative / maxSplineVel)
	// / calcDivisor(curvature)); // Or flip signs? equivalent? Also, make better by finding min val of maxAccel across the segment by making all of these (including V of robot?) functions of s and solving or probably just check at various values of s
	// double max2 = (robotMaxAccel - Math.signum(curvature) * (plantWidth * maxVel * maxVel * curveDerivative / maxSplineVel)
	// / calcDivisor(-curvature));
	// if (max1 < 2) {
	// return max1;
	// } else {
	// return max2;
	// }
	// }
	public double calcMaxVel(double curvature) {
		return robotMaxVel / calcDivisor(curvature);
	}

	private double calcDivisor(double curvature) {
		return 1 - Math.signum(curvature) * (plantWidth * Math.abs(curvature));
	}

	public int getTickTotal() {
		return tickTotal;
	}

	public double getTickTime() {
		return tickTime;
	}
}