package io.getcoffee.motionprofiles;


import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

strictfp public class WheelTrajectory {
	protected final MotionTrajectory motionTrajectoryProfile;
	protected LinkedList<WheelTrajectorySegment> trajectorySegments;
	protected final Wheel wheel;
	protected final double tickTotal;
	protected final double tickTime;

	public static enum Wheel {
		LEFT(-1), RIGHT(1);
		private final int modifier;

		Wheel(int modifier) {
			this.modifier = modifier;
		}

		public int getModifier() {
			return modifier;
		}
	}

	public WheelTrajectory(MotionTrajectory motionTrajectoryProfile, LinkedList<SplineSegment> featureSegments, Wheel wheel,
		double tickTotal, double tickTime) {
		this.motionTrajectoryProfile = motionTrajectoryProfile;
		this.wheel = wheel;
		this.tickTotal = tickTotal;
		this.tickTime = tickTime;
		trajectorySegments = finalizeSegments(generateBackwardConsistency(generateForwardConsistency(featureSegments)));
	}

	/**
	 * Generates a 'forward consistent' set of segments for each spline feature segment. This ensures that
	 * the movement follows the acceleration constraints of both the robot and path.
	 * 
	 * The segments that this returns are not ready to be used and should be run through the backward consistency
	 * filter and afterwards finalized.
	 * 
	 * @see {@link WheelTrajectory#generateBackwardConsistency}
	 * @see {@link WheelTrajectory#finalizeSegments}
	 * 
	 * @param featureSegments
	 * @return ordered right/left trajectory segments that are forward consistent.
	 */
	public LinkedList<WheelTrajectorySegment> generateForwardConsistency(LinkedList<SplineSegment> featureSegments) {
		LinkedList<WheelTrajectorySegment> trajectorySegments = new LinkedList<>();
		WheelTrajectorySegment lastSegment = new WheelTrajectorySegment(0, calcMaxVel(0));
		for (int i = 0; i < featureSegments.size(); i++) {
			double newMaxVel = calcMaxVelFromCurvature(featureSegments.get(i).calcMaxCurvature());
			lastSegment.length = calcLength(featureSegments.get(i).initPercentage, featureSegments.get(i).finPercentage);
			lastSegment.finVel = Math.min(lastSegment.calcReachableEndVel(), Math.min(lastSegment.maxVel, newMaxVel));
			trajectorySegments.add(lastSegment);
			if (i != featureSegments.size() - 1) {
				lastSegment = new WheelTrajectorySegment(lastSegment.finVel,
					newMaxVel);
			}
		}
		lastSegment.finVel = 0;
		return trajectorySegments;
	}

	/**
	 * Generates a 'backward consistent' set of segments from a 'forward consistent' set of segments. Establishing
	 * backwards consistency ensures the segments obey deceleration constraints.
	 * 
	 * @param trajectorySegments
	 *        ordered segments that are forward consistent
	 * @return ordered right/left trajectory segments that are forward and backward consistent.
	 */
	public LinkedList<WheelTrajectorySegment> generateBackwardConsistency(
		LinkedList<WheelTrajectorySegment> trajectorySegments) {
		for (int i = trajectorySegments.size() - 1; i > 0; i--) {
			trajectorySegments.get(i).initVel = trajectorySegments.get(i).calcReachableStartVel();
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
	public LinkedList<WheelTrajectorySegment> finalizeSegments(LinkedList<WheelTrajectorySegment> trajectorySegments) {
		double timePassed = 0;
		double distanceTraveled = 0;
		for (WheelTrajectorySegment segment : trajectorySegments) {
			segment.dividePath();
			segment.context = new AbsoluteSegmentContext(timePassed, distanceTraveled);
			timePassed += segment.duration;
			distanceTraveled += segment.length;
		}
		return trajectorySegments;
	}

	/**
	 * Generates a map of each of the possible ticks, and maps each one to the segment during which
	 * that tick occurs, and the time during the segment that the tick occurs at.
	 * 
	 * The generation herein potentially ignores any segments shorter than the tick time.
	 * 
	 * @return Map<Tick, Tuple<Time of tick occurrence, Trajectory Segment that the tick happens during>>
	 */
	public Map<Integer, Tuple<Double, MotionTrajectorySegment>> generateTickMap() {
		HashMap<Integer, Tuple<Double, MotionTrajectorySegment>> map = new HashMap<>();
		Integer currentSegmentIndex = 0;
		double currentSegmentDuration = trajectorySegments.get(currentSegmentIndex).duration;
		double timeOverSegment = 0.0;
		for (Integer i = 0; i < tickTotal; i++) {
			double timeDiff = (timeOverSegment += tickTime) - currentSegmentDuration;
			if (timeDiff > 0) {
				timeOverSegment = timeDiff;
				currentSegmentIndex++;
				currentSegmentDuration = trajectorySegments.get(currentSegmentIndex).duration;
			}
			map.put(i, new Tuple<>(timeOverSegment, trajectorySegments.get(currentSegmentIndex)));
		}
		return map;
	}

	public double calcMaxVel(double s) {
		return motionTrajectoryProfile.calcMaxSpeed(s)
			+ (motionTrajectoryProfile.calcMaxAngularVel(s) * wheel.getModifier()) / 2.0;
	}

	public double calcMaxVelFromCurvature(double curvature) {
		double maxTranslational = motionTrajectoryProfile.calcMaxSpeedFromCurvature(curvature);
		return maxTranslational
			+ (motionTrajectoryProfile.calcAngularVel(maxTranslational, curvature) * wheel.getModifier()) / 2.0;
	}

	public double calcAcc(double s, MotionTrajectoryPoint lastPoint) {
		return (calcMaxVel(s) - lastPoint.vel) / (1 / tickTotal);
	}

	public double calcBadLength(double length, double curvature) {
		return length * ((curvature * motionTrajectoryProfile.plantWidth + wheel.getModifier()) / 2);
	}

	public double calcLength(double a, double b, double granularity) {
		double arcSum = 0;
		for (double i = a; i < b; i += 1 / granularity) {
			arcSum += calcPathSpeed(i);
		}
		return arcSum / granularity;
	}

	public double calcLength(double a, double b) {
		return calcLength(a, b, SplineGenerator.INTEGRATION_GRANULARITY * 2);
	}

	public double calcPathSpeed(double s) {
		Tuple<Tuple<Double, Double>, Tuple<Double, Double>> info = motionTrajectoryProfile.calcPerpDerivativeAndSpeed(s);
		Tuple<Double, Double> perpD = info.getX();
		Tuple<Double, Double> splineVel = info.getY();
		double speed = Math.sqrt(
			(splineVel.getX() + perpD.getX() * wheel.getModifier()) * (splineVel.getX() + perpD.getX() * wheel.getModifier())
				+ (splineVel.getY() + perpD.getY() * wheel.getModifier())
					* (splineVel.getY() + perpD.getY() * wheel.getModifier())); // derivative of the wheel position (splinePosition +- perpVector) = splinePos' +- perpVector'
		return speed;
	}
}
