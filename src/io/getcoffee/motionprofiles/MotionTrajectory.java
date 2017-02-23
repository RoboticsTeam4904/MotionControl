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
	public MotionTrajectory(SplineGenerator splineGenerator, double plantWidth, double tickTime) {
		this.splineGenerator = splineGenerator;
		this.plantWidth = plantWidth / 2.0;
		this.tickTime = tickTime;
		// TODO: Update the threshold to reflect a real value.
		featureSegments = splineGenerator.generateFeatureSegments(0.1);
		trajectorySegments = finalizeSegments(
			generateBackwardConsistency(generateSegmentsAndForwardConsistency(featureSegments)));
		tickMap = generateFullTickMap(trajectorySegments);
	}

	public LinkedList<MotionTrajectorySegment> generateIsolatedSegments(LinkedList<SplineSegment> featureSegments) {
		LinkedList<MotionTrajectorySegment> trajectorySegments = new LinkedList<>();
		MotionTrajectorySegment lastSegment = new MotionTrajectorySegment(0.0);
		for(SplineSegment featureSegment : featureSegments) {
			last
		}
		return trajectorySegments;
	}
	
	/**
	 * Generates a 'forward consistent' set of segments for each spline feature segment. This ensures that
	 * the movement follows the acceleration constraints of both the robot and path.
	 * 
	 * The segments that this returns are not ready to be used and should be run through the backward consistency
	 * filter and afterwards finalized.
	 * 
	 * @see {@link MotionTrajectory#generateBackwardConsistency}
	 * @see {@link MotionTrajectory#finalizeSegments}
	 * 
	 * @param featureSegments
	 * @return ordered right/left trajectory segments that are forward consistent.
	 */
	public LinkedList<MotionTrajectorySegment> generateSegmentsAndForwardConsistency(
		LinkedList<SplineSegment> featureSegments) {
		LinkedList<MotionTrajectorySegment> trajectorySegments = new LinkedList<>();
		MotionTrajectorySegment lastSegment = new MotionTrajectorySegment(0.0); // Initialize velocity at 0
		Tuple<Double, Double> maxVelAndAccel = calcMaxVelAndAccFromCurvatureAndDerivative(featureSegments.get(0).maxCurve,
			featureSegments.get(0).maxCurveDerivative);
		for (SplineSegment featureSegment : featureSegments) {
			lastSegment = new MotionTrajectorySegment(featureSegment.length, lastSegment.finVel, maxVelAndAccel.getX(),
				maxVelAndAccel.getY());
			maxVelAndAccel = calcMaxVelAndAccFromCurvatureAndDerivative(featureSegment.maxCurve,
				featureSegment.maxCurveDerivative);
			lastSegment.finVel = Math.min(lastSegment.calcReachableEndVel(),
				Math.min(lastSegment.maxVel, maxVelAndAccel.getX()));
			trajectorySegments.add(lastSegment);
		}
		lastSegment.finVel = 0;
	}

	/**
	 * Generates a 'backward consistent' set of segments from a 'forward consistent' set of segments. Establishing
	 * backwards consistency ensures the segments obey deceleration constraints.
	 * 
	 * @param trajectorySegments
	 *        ordered segments that are forward consistent
	 * @return ordered right/left trajectory segments that are forward and backward consistent.
	 */
	public LinkedList<MotionTrajectorySegment> generateBackwardConsistency(
		LinkedList<MotionTrajectorySegment> trajectorySegments) {
		for (int i = trajectorySegments.size() - 1; i > 0; i--) {
			MotionTrajectorySegment trajectorySegment = trajectorySegments.get(i);
			trajectorySegment.initVel = Math.min(trajectorySegment.calcReachableStartVel(), trajectorySegment.initVel);
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
			for (;timeOverSegment < segment.duration; timeOverSegment += tickTime) {
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
		Tuple<Double, MotionTrajectorySegment> leftWheelTick = map.get(tick);
		MotionTrajectoryPoint generalSetpoint = tickMap.get(tick);
		splineGenerator.calcCurvature(generalSetpoint.pos);
		// Calc w by finding curvature(pos) where pos is arclength and multiplying by width and v. v is setpoint in fullmap. calc indiv wheel vels from this?

		return new Tuple<MotionTrajectoryPoint, MotionTrajectoryPoint>(
			leftWheelTick.getY().findSetPoint(leftWheelTick.getX(), tick),
			rightWheelTick.getY().findSetPoint(rightWheelTick.getX(), tick));
	}

	/**
	 * Super constraining right now. Uses maxVel, maxCurvature and maxCurvatureD.
	 * It would ideally find the minAccel given the maxCurvature and maxCurvatureD at every point
	 * As well as vel at that point as a function of maxVel and accel itself (simultaneous calculation possible?)
	 * 
	 * @param curvature
	 * @param dCurvature
	 * @return
	 */
	public Tuple<Double, Double> calcMaxAccelFromCurvatureAndDerivative(double curvature, double dCurvature) {
		double divisor = (1 + plantWidth * Math.abs(curvature));
		double maxVel = robotMaxVel / divisor;
		return new Tuple<>(maxVel, (robotMaxAccel - plantWidth * maxVel * dCurvature) / divisor);
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

	public int getTickTotal() {
		return tickTotal;
	}
}