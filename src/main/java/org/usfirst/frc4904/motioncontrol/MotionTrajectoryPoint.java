package org.usfirst.frc4904.motioncontrol;


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
		return "MotionTrajectoryPoint#{Tick: " + tick + ", Pos: " + pos + ", Vel: " + vel + ", Accel: " + accel + "}";
	}
}
