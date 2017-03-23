package io.getcoffee.motionprofiles;


public class MotionTrajectorySegment {
	protected double initVel;
	protected double finVel;
	protected double length;
	protected double maxVel;
	protected double maxAccel;
	protected double minAccel;
	protected double duration;
	protected double adjustedMaxVel;
	protected double rampUpTime;
	protected double rampDownTime;
	protected double rampUpDistance;
	protected double rampDownDistance;
	protected double cruiseDistance;
	protected double cruiseTime;

	public MotionTrajectorySegment(double length, double initVel, double maxVel, double maxAccel, double minAccel) {
		this.length = length;
		this.initVel = initVel;
		this.maxVel = maxVel;
		this.maxAccel = maxAccel;
		this.minAccel = minAccel;
	}

	public double calcVelFromFrontAndBack(double distance) {
		return Math.min(maxVel, Math.min(calcMaxReachableVelForward(distance), calcMaxReachableVelBack(distance)));
	}

	private double calcMaxReachableVelForward(double distance) {
		return Math.sqrt(2 * maxAccel * distance + (initVel * initVel));
	}

	private double calcMaxReachableVelBack(double distance) {
		return Math.sqrt(-2 * minAccel * (length - distance) + (finVel * finVel));
	}

	public double calcReachableEndVel() {
		return calcMaxReachableVelForward(length);
	}

	public double calcReachableStartVel() {
		return calcMaxReachableVelBack(0);
	}

	public double calcAdjustedVel() {
		return Math.sqrt(maxAccel * (-2 * minAccel * length + (finVel * finVel)) - minAccel * (initVel * initVel))
			/ Math.sqrt(maxAccel - minAccel);
	}

	protected double Pos(double t, double initVel, double a) {
		return initVel * t + (a * t * t) / 2;
	}

	protected double Vel(double t, double initVel, double a) {
		return initVel + a * t;
	}

	protected void dividePath() {
		adjustedMaxVel = Math.min(maxVel, calcAdjustedVel());
		rampUpTime = (adjustedMaxVel - initVel) / maxAccel;
		rampDownTime = (finVel - adjustedMaxVel) / minAccel;
		rampUpDistance = (adjustedMaxVel * adjustedMaxVel - initVel * initVel)
			/ (2 * maxAccel);
		rampDownDistance = (finVel * finVel - adjustedMaxVel * adjustedMaxVel)
			/ (2 * minAccel);
		cruiseDistance = length - rampUpDistance - rampDownDistance;
		cruiseTime = cruiseDistance / adjustedMaxVel;
		duration = rampUpTime + rampDownTime + cruiseTime;
	}

	/**
	 * Calculates the setpoint for a tick and time
	 *
	 * @param t
	 * @param tick
	 * @param posOffset
	 *        the amount to offset the position of the setpoint by
	 * @return Relative setpoint with position offset by absolute context's distance.
	 */
	protected MotionTrajectoryPoint calcOffsetSetpoint(double t, int tick, double posOffset) {
		MotionTrajectoryPoint relativeSetPoint = calcSetpoint(t, tick);
		relativeSetPoint.pos += posOffset;
		return relativeSetPoint;
	}

	protected MotionTrajectoryPoint calcSetpoint(double t, int tick) {
		double pos;
		double vel;
		double accel;
		if (t <= rampUpTime) {
			pos = 0.0;
			vel = initVel;
			accel = maxAccel;
		} else if (t <= rampUpTime + cruiseTime) {
			pos = rampUpDistance;
			vel = adjustedMaxVel;
			accel = 0.0;
			t -= rampUpTime;
		} else {
			pos = rampUpDistance + cruiseDistance;
			vel = adjustedMaxVel;
			accel = minAccel;
			t -= rampUpTime + cruiseTime;
		}
		vel = Vel(t, vel, accel);
		pos += Pos(t, vel, accel);
		return new MotionTrajectoryPoint(tick, pos, vel, accel);
	}

	@Override
	public String toString() {
		return "MotionTrajectorySegment#{InitVel: " + initVel +
				", FinVel: " + finVel +
				", MaxVel: " + maxVel +
				", MinAccel: " + minAccel +
				", MaxAccel: " + maxAccel +
				", Length: " + length +
				", Duration: " + duration + "}";
	}
}
