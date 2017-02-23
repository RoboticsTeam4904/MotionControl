package io.getcoffee.motionprofiles;


public class MotionTrajectoryPoint {
	public final int tick;
	public double pos;
	public double vel;
	public double accel;

	public MotionTrajectoryPoint(int tick, double pos, double vel, double accel) {
		this.tick = tick;
		this.pos = pos;
		this.vel = vel;
		this.accel = accel;
	}

	@Override
	public String toString() {
		return "MP#" + tick + "{" + pos + ", " + vel + ", " + accel + "}";
	}
}
