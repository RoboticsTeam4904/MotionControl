package org.usfirst.frc4904.motioncontrol;


import java.util.Deque;
import java.util.concurrent.LinkedBlockingDeque;

public class MotionTrajectoryQueue {
	protected final MotionTrajectory trajectory;
	public LinkedBlockingDeque<Tuple<MotionTrajectoryPoint, MotionTrajectoryPoint>> pointQueue;
	protected MotionTrajectoryBuilder pointQueueBuilder;
	protected Thread pointQueueBuilderThread;

	public MotionTrajectoryQueue(MotionTrajectory trajectory) {
		this.trajectory = trajectory;
		pointQueue = new LinkedBlockingDeque<>();
		pointQueueBuilder = new MotionTrajectoryBuilder(trajectory, pointQueue);
		pointQueueBuilderThread = new Thread(pointQueueBuilder);
	}

	public void build() {
		this.pointQueueBuilderThread.start();
	}

	public void cancel() {
		this.pointQueueBuilderThread.interrupt();
	}

	public MotionTrajectory getTrajectory() {
		return trajectory;
	}

	protected class MotionTrajectoryBuilder implements Runnable {
		private final MotionTrajectory trajectory;
		private final LinkedBlockingDeque<Tuple<MotionTrajectoryPoint, MotionTrajectoryPoint>> queue;
		private final Tuple<MotionTrajectoryPoint, MotionTrajectoryPoint> zeroPoint = new Tuple<>(
			new MotionTrajectoryPoint(0, 0, 0, 0), new MotionTrajectoryPoint(0, 0, 0, 0));

		MotionTrajectoryBuilder(MotionTrajectory trajectory,
			LinkedBlockingDeque<Tuple<MotionTrajectoryPoint, MotionTrajectoryPoint>> queue) {
			this.trajectory = trajectory;
			this.queue = queue;
			queue.add(zeroPoint);
		}

		@Override
		public void run() {
			Tuple<MotionTrajectoryPoint, MotionTrajectoryPoint> lastPoints = zeroPoint;
			int tickNum = 0;
			while(true) {
				lastPoints = trajectory.calcPoint(tickNum += 1, lastPoints);
				queue.add(lastPoints);
			}
		}
	}
}
