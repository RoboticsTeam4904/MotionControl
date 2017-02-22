package io.getcoffee.motionprofiles;

public class MotionTrajectorySegment {
	protected double initVel;
	protected double finVel;
	protected double length;
	protected double maxVel;
	protected double maxAccel;
	protected double duration;
	protected double adjustedMaxVel;
	protected double rampUpTime;
	protected double rampDownTime;
	protected double rampUpDistance;
	protected double rampDownDistance;
	protected double cruiseDistance;
	protected double cruiseTime;
	public AbsoluteSegmentContext context;

	public MotionTrajectorySegment(double length, double initVel, double maxVel, double maxAccel) {
		this.length = length;
		this.initVel = initVel;
		this.maxVel = maxVel;
		this.maxAccel = maxAccel;
	}

	public double calcVelFromFrontAndBack(double distance) {
		return Math.min(maxVel, Math.min(maxReachableVel(length, initVel), maxReachableVel(length - distance, initVel)));
	}

	private double maxReachableVel(double distance, double initVel) {
		return Math.sqrt(2 * maxAccel * distance + initVel * initVel);
	}

	public double calcReachableEndVel() {
		return maxReachableVel(length, initVel);
	}

	public double calcReachableStartVel() {
		return maxReachableVel(length, finVel);
	}

	protected double Pos(double t, double initVel, double a) {
		return initVel * t + (a * t * t) / 2;
	}

	protected double Vel(double t, double initVel, double a) {
		return initVel + a * t;
	}

	public double calcAdjustedVel() {
		return Math.sqrt(maxAccel * length + (initVel * initVel + finVel * finVel) / 2);
	}

	protected void dividePath() {
		adjustedMaxVel = Math.min(maxVel, calcAdjustedVel());
		rampUpTime = (adjustedMaxVel - initVel) / maxAccel;
		rampDownTime = (adjustedMaxVel - finVel) / maxAccel;
		rampUpDistance = (adjustedMaxVel * adjustedMaxVel - initVel * initVel)
			/ (2 * maxAccel);
		rampDownDistance = (adjustedMaxVel * adjustedMaxVel - finVel * finVel)
			/ (2 * maxAccel);
		cruiseDistance = length - rampUpDistance - rampDownDistance;
		cruiseTime = cruiseDistance / adjustedMaxVel;
		duration = rampUpTime + rampDownTime + cruiseTime;
	}

	protected MotionTrajectoryPoint findSetPoint(double t, int tick) {
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
			accel = -maxAccel;
			t -= rampUpTime + cruiseTime;
		}
		vel = Vel(t, vel, accel);
		pos += Pos(t, vel, accel);
		return new MotionTrajectoryPoint(tick, context.absoluteDistance + pos, vel, accel);
	}
	
	@Override
	public String toString() {
		return "MotionSegment#{InitVel: "+ initVel + ", FinVel: " + finVel + ", MaxVel: " + maxVel + ", Length: " + length + ", Duration: " + duration + "}";
	}
}
