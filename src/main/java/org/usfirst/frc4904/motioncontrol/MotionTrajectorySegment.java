package org.usfirst.frc4904.motioncontrol;


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
		System.out.println("Length: " + length + "; Initial Velocity: " + initVel + "; Max Velocity: " + maxVel
			+ "; Max Acceleration: " + maxAccel + "; Min Acceleration: " + minAccel);
	}

	public MotionTrajectorySegment(double maxVel) {
		this.maxVel = maxVel;
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
		System.out.println("Max Accel: " + maxAccel);
		System.out.println("Min Accel: " + minAccel);
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
		System.out.println("Max Velocity: " + maxVel);
		System.out.println("Adjusted Velocity Calc: " + calcAdjustedVel());
		adjustedMaxVel = Math.min(maxVel, calcAdjustedVel());
		System.out.println("Adjusted Max Velocity: " + adjustedMaxVel);
		rampUpTime = (adjustedMaxVel - initVel) / maxAccel;
		System.out.println("Ramp Up Time: " + rampUpTime);
		rampDownTime = (finVel - adjustedMaxVel) / minAccel;
		System.out.println("Ramp Down Time: " + rampDownTime);
		rampUpDistance = (adjustedMaxVel * adjustedMaxVel - initVel * initVel)
			/ (2 * maxAccel);
		System.out.println("Ramp Up Distance: " + rampUpDistance);
		rampDownDistance = (finVel * finVel - adjustedMaxVel * adjustedMaxVel)
			/ (2 * minAccel);
		System.out.println("Ramp Down Distance: " + rampDownDistance);
		cruiseDistance = length - rampUpDistance - rampDownDistance;
		System.out.println("Cruise Distance: " + cruiseDistance);
		cruiseTime = cruiseDistance / adjustedMaxVel;
		System.out.println("Cruise Time: " + cruiseTime);
		duration = rampUpTime + rampDownTime + cruiseTime;
		System.out.println("Duration: " + duration);
	}

	/**
	 * Calculates the setpoint for a tick and time
	 *
	 * @param t
	 * @param tick
	 * @param posOffset
	 *                  the amount to offset the position of the setpoint by
	 * @return Relative setpoint with position offset by absolute context's distance.
	 */
	protected MotionTrajectoryPoint calcOffsetSetpoint(double t, int tick, double posOffset) {
		MotionTrajectoryPoint relativeSetPoint = calcSetpoint(t, tick);
		relativeSetPoint.pos += posOffset;
		return relativeSetPoint;
	}

	protected MotionTrajectoryPoint calcSetpoint(double t, int tick) {
		double init_pos;
		double init_vel;
		double accel;
		if (t <= rampUpTime) {
			init_pos = 0.0;
			init_vel = initVel;
			accel = maxAccel;
		} else if (t <= rampUpTime + cruiseTime) {
			init_pos = rampUpDistance;
			init_vel = adjustedMaxVel;
			accel = 0.0;
			t -= rampUpTime;
		} else {
			init_pos = rampUpDistance + cruiseDistance;
			init_vel = adjustedMaxVel;
			accel = minAccel;
			t -= rampUpTime + cruiseTime;
		}
		double vel = Vel(t, init_vel, accel);
		double pos = init_pos + Pos(t, init_vel, accel);
		// System.out.println(init_pos + "," + init_vel + "," + accel + ", t:" + t + "\t" + rampUpTime + ",\t" + cruiseTime + ",\t" + rampDownTime + ",\t");
		// System.out.println(pos + "," + vel + "," + accel + ", t:" + tick + "\t" + Pos(t, init_vel, accel) + "," + Vel(t, init_vel, accel));
		return new MotionTrajectoryPoint(tick, pos, vel, accel);
	}

	@Override
	public String toString() {
		// return "MotionTrajectorySegment#{InitVel: " + initVel + ", FinVel: " + finVel + ", MaxVel: " + maxVel + ", MinAccel: "
		// + minAccel + ", MaxAccel: "
		// + maxAccel + ", Length: " + length
		// + ", Duration: " + duration + "}";
		return "MotionTrajectorySegment[duration=" + duration + ", adjustedMaxVel=" + adjustedMaxVel + "]";
	}

	// TODO: Is needed? Is correct?
	public double calcDuration() {
		return (minAccel * (maxVel - initVel) * (maxVel - initVel)
			+ maxAccel * (2 * minAccel * length - (maxVel - finVel) * (maxVel - finVel))) / (2 * maxAccel * minAccel * maxVel);
	}
}
