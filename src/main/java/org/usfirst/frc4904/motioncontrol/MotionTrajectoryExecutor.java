package org.usfirst.frc4904.motioncontrol;


import java.io.FileWriter;
import java.io.IOException;
import org.usfirst.frc4904.motioncontrol.MotionTrajectory;
import org.usfirst.frc4904.motioncontrol.MotionTrajectoryPoint;
import org.usfirst.frc4904.motioncontrol.Tuple;
import org.usfirst.frc4904.motioncontrol.pathing.PathGenerator;
import org.usfirst.frc4904.motioncontrol.pathing.SplineGenerator;

strictfp public class MotionTrajectoryExecutor {
	public static final double robotMaxVel = 6;
	public static final double robotMaxAccel = 6;
	public static final double plantWidth = 0.25; // radius

	public static void main(String[] args) {
		PathGenerator spline = SplineGenerator.fitQuintic(
			// xi, yi, xf, yf
			// Position
			0, 0, 5, 5,
			// Velocity
			10, 0, 10, 0,
			// Acceleration
			1, 0, -1, 0);
		// PathGenerator spline = new CirclePathGenerator(0, 0, 1, 2, -0.2);
		// testSpline(0.6, spline);
		// System.out.println(spline.calcSpeed(0.975));
		MotionTrajectory motionTrajectory = new MotionTrajectory(spline, plantWidth, 20);
		printPoints(motionTrajectory);
	}

	public static void printPoints(MotionTrajectory motionTrajectory) {
		System.out.println("Tick total: " + motionTrajectory.getTickTotal());
		try (FileWriter csvWriter = new FileWriter("new.csv")) {
			csvWriter.append("x,y,theta\n");
			// csvWriter.append("Left_Tick,Left_Pos,Left_Vel,Left_Accel,Right_Tick,Right_Pos,Right_Vel,Right_Accel\n");
			System.out.println("Left Tick, Left Pos, Left Vel, Left Accel, Right Tick, Right Pos, Right Vel, Right Accel");
			MotionTrajectoryPoint motionTrajectoryPoint = new MotionTrajectoryPoint(0, 0, 0, 0);
			Tuple<MotionTrajectoryPoint, MotionTrajectoryPoint> lastPoint = new Tuple<>(motionTrajectoryPoint,
				motionTrajectoryPoint);
			double x = 0;
			double y = 0;
			double theta = 0;
			double d = plantWidth;
			for (int i = 0; i < motionTrajectory.getTickTotal(); i++) {
				Tuple<MotionTrajectoryPoint, MotionTrajectoryPoint> newPoint = motionTrajectory.calcPoint(i, lastPoint);
				MotionTrajectoryPoint left = newPoint.getX();
				MotionTrajectoryPoint right = newPoint.getY();
				double vl = left.vel;
				double vr = right.vel;
				double v = (vl + vr) / 2;
				double w = (vr - vl) / d;
				w = w / 2;
				theta += w * .02;
				x += v * Math.cos(theta) * .02;
				y += v * Math.sin(theta) * .02;
				System.out.println("(" + x + ", " + y + ", " + theta + ")");
				csvWriter.append(x + "," + y + "," + theta + "\n");
				// System.out.println(left.tick + "," + left.pos + "," + left.vel + "," + left.accel + "," + right.tick + ","
				// + right.pos + "," + right.vel + "," + right.accel);
				// csvWriter.append(left.tick + "," + left.pos + "," + left.vel + "," + left.accel + "," + right.tick + ","
				// + right.pos + "," + right.vel + "," + right.accel + "\n");
				lastPoint = newPoint;
			}
			csvWriter.flush();
			csvWriter.close();
		}
		catch (IOException e) {
			System.out.println(e);
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
