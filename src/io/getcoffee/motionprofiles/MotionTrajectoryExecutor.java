package io.getcoffee.motionprofiles;


strictfp public class MotionTrajectoryExecutor {
	public static final double robotMaxVel = 5;
	public static final double robotMaxAccel = 1;

	public static void main(String[] args) {
		SplineGenerator spline = new QuinticSplineGenerator(
			// Position
			0, 0, 1, 2,
			// Velocity
			1, 0, 0, 1,
			// Acceleration
			1, 0, 0, -20);
		System.out.println(spline.calcPos(0.6));
		System.out.println(spline.calcVel(0.6));
		System.out.println(spline.calcAcc(0.6));
		System.out.println(spline.calcSpeed(0.6));
		System.out.println(spline.calcCurvature(0.6));
		System.out.println("##############################");
		MotionTrajectory motionTrajectory = new MotionTrajectory(spline, 0.5, 10);
		System.out.println(motionTrajectory.featureSegments.size());
		System.out.println(motionTrajectory.rightWheel.trajectorySegments.size());
		System.out.println(motionTrajectory.leftWheel.trajectorySegments.size());
		System.out.println("##############################");
		double durationTallyRight = 0;
		double durationTallyLeft = 0;
		double lengthTallyRight = 0;
		double lengthTallyLeft = 0;
		for (int i = 0; i < motionTrajectory.featureSegments.size() - 1; i++) {
			System.out.println("RIGHT{" + i + "}" + motionTrajectory.rightWheel.trajectorySegments.get(i).duration + " LEFT{"
				+ i + "}" + motionTrajectory.leftWheel.trajectorySegments.get(i).duration + " DIF # "
				+ (motionTrajectory.rightWheel.trajectorySegments.get(i).duration
					- motionTrajectory.leftWheel.trajectorySegments.get(i).duration));
			// System.out.println(motionTrajectory.leftWheel.trajectorySegments.get(i));
			durationTallyRight += motionTrajectory.rightWheel.trajectorySegments.get(i).duration;
			durationTallyLeft += motionTrajectory.leftWheel.trajectorySegments.get(i).duration;
			lengthTallyRight += motionTrajectory.rightWheel.trajectorySegments.get(i).length;
			lengthTallyLeft += motionTrajectory.leftWheel.trajectorySegments.get(i).length;
		}
		System.out.println("RIGHT DUR: " + (durationTallyRight / motionTrajectory.featureSegments.size()));
		System.out.println("LEFT  DUR: " + (durationTallyLeft / motionTrajectory.featureSegments.size()));
		System.out.println("RIGHT LEN: " + (lengthTallyRight / motionTrajectory.featureSegments.size()));
		System.out.println("LEFT  LEN: " + (lengthTallyLeft / motionTrajectory.featureSegments.size()));
		// System.out.println(motionTrajectory.calcPoint(0.8, lastPoint));
		MotionTrajectoryPoint motionTrajectoryPoint = new MotionTrajectoryPoint(0, 0, 0, 0);
		Tuple<MotionTrajectoryPoint, MotionTrajectoryPoint> lastPoint = new Tuple<>(motionTrajectoryPoint,
			motionTrajectoryPoint);
		for (int i = 1; i < 200; i++) {
			System.out.println(lastPoint);
			Tuple<MotionTrajectoryPoint, MotionTrajectoryPoint> newPoint = motionTrajectory.calcPoint(i);
			lastPoint = newPoint;
		}
		System.out.println(lastPoint);
		{}
	}
}
