package io.getcoffee.motionprofiles.pathing;


import java.util.TreeMap;

public abstract class PathGenerator {
	public double absoluteLength;
	public static final double INTEGRATION_GRANULARITY = 100;
	public TreeMap<Double, PathSegment> featureSegmentMap = new TreeMap<>();

	protected abstract void initialize(double threshold, double granularity);

	protected void initialize(double threshold) {
		initialize(threshold, PathGenerator.INTEGRATION_GRANULARITY);
	}

	public abstract double calcCurvature(double s);

	protected abstract double PosX(double s);

	protected abstract double PosY(double s);

	protected abstract double VelX(double s);

	protected abstract double VelY(double s);

	protected abstract double AccX(double s);

	protected abstract double AccY(double s);

	protected abstract double JerkX(double s);

	protected abstract double JerkY(double s);
}
