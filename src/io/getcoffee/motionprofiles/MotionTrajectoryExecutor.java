package io.getcoffee.motionprofiles;


strictfp public class MotionTrajectoryExecutor {
	public static final double robotMaxVel = 5;
	public static final double robotMaxAccel = 10;

	public static void main(String[] args) {
		SplineGenerator spline = new QuinticSplineGenerator(
			// xi, yi, xf, yf
			// Position
			0, 0, 1, 2,
			// Velocity
			1, 0, 0, 1,
			// Acceleration
			1, 0, 0, -20);
		testSpline(0.6, spline);
		MotionTrajectory motionTrajectory = new MotionTrajectory(spline, 0.5, 10); // where do we get tick time?
		printPoints(motionTrajectory);
	}

	public static void printPoints(MotionTrajectory motionTrajectory) {
		MotionTrajectoryPoint motionTrajectoryPoint = new MotionTrajectoryPoint(0, 0, 0, 0);
		Tuple<MotionTrajectoryPoint, MotionTrajectoryPoint> lastPoint = new Tuple<>(motionTrajectoryPoint,
			motionTrajectoryPoint);
		for (int i = 1; i < motionTrajectory.getTickTotal(); i++) { // What about the first setpoint?
			System.out.println(i); // last point should be used as context for new point (makes program run better and more efficiently, alternative not implemented) (though honestly the contextualized method hasn't been implemented either) (well at least we thought about it a lot)
			Tuple<MotionTrajectoryPoint, MotionTrajectoryPoint> newPoint = motionTrajectory.calcPoint(i, lastPoint);
			lastPoint = newPoint;
		}
		System.out.println(lastPoint);
	}

	public static void testSpline(double s, SplineGenerator spline) {
		System.out.println("Spline point at " + s);
		System.out.println("Pos " + spline.calcPos(s));
		System.out.println("Vel " + spline.calcVel(s));
		System.out.println("Acc " + spline.calcAcc(s));
		System.out.println("Spd " + spline.calcSpeed(s));
		System.out.println("Curv " + spline.calcCurvature(s));
	}
}
