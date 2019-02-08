package org.usfirst.frc4904.motioncontrol;

import org.usfirst.frc4904.motioncontrol.MotionTrajectory;
import org.usfirst.frc4904.motioncontrol.MotionTrajectoryPoint;
import org.usfirst.frc4904.motioncontrol.Tuple;
import org.usfirst.frc4904.motioncontrol.pathing.PathGenerator;
import org.usfirst.frc4904.motioncontrol.pathing.SplineGenerator;

strictfp public class MotionTrajectoryExecutor {
	public static final double robotMaxVel = 5;
	public static final double robotMaxAccel = 1;
	public static final double plantWidth = 0.25; // radius

	public static void main(String[] args) {
		PathGenerator spline = SplineGenerator.fitQuintic(
				// xi, yi, xf, yf
				// Position
				0, 0, 1, 2,
				// Velocity
				1, 0, 0, 1,
				// Acceleration
				1, 0, 0, -20);
		// PathGenerator spline = new CirclePathGenerator(0, 0, 1, 2, -0.2);
//		testSpline(0.6, spline);
//		System.out.println(spline.featureSegmentMap);
		MotionTrajectory motionTrajectory = new MotionTrajectory(spline, plantWidth, 10);
//		printPoints(motionTrajectory);
	}

	public static void printPoints(MotionTrajectory motionTrajectory) {
		System.out.println("Left Tick, Left Pos, Left Vel, Left Accel, Right Tick, Right Pos, Right Vel, Right Accel");
		MotionTrajectoryPoint motionTrajectoryPoint = new MotionTrajectoryPoint(0, 0, 0, 0);
		Tuple<MotionTrajectoryPoint, MotionTrajectoryPoint> lastPoint = new Tuple<>(motionTrajectoryPoint,
				motionTrajectoryPoint);
		for (int i = 0; i < motionTrajectory.getTickTotal(); i++) {
			Tuple<MotionTrajectoryPoint, MotionTrajectoryPoint> newPoint = motionTrajectory.calcPoint(i, lastPoint);
			MotionTrajectoryPoint left = newPoint.getX();
			MotionTrajectoryPoint right = newPoint.getY();
			System.out.println(left.tick + "," + left.pos + "," + left.vel + "," + left.accel + "," + right.tick + "," + right.pos + "," + right.vel + "," + right.accel);
			lastPoint = newPoint;
		}
	}

	public static void testSpline(double s, PathGenerator spline) {
		System.out.println("Path point at " + s);
		System.out.println("Pos " + spline.calcPos(s));
		System.out.println("Vel " + spline.calcVel(s));
		System.out.println("Acc " + spline.calcAcc(s));
		System.out.println("Spd " + spline.calcSpeed(s));
		System.out.println("Curv " + spline.calcCurvature(s));
	}
}
