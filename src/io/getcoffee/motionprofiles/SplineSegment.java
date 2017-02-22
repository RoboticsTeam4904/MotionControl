package io.getcoffee.motionprofiles;


public class SplineSegment {
	protected double initCurve;
	protected double finCurve;
	protected double maxCurve;
	public double initPercentage;
	protected double finPercentage;
	protected double length;

	public SplineSegment(double initCurve, double finCurve, double maxCurve, double length) {
		this.initCurve = initCurve;
		this.finCurve = finCurve;
		this.maxCurve = maxCurve;
		this.length = length;
	}

	public SplineSegment(double initCurve) {
		this.initCurve = initCurve;
	}

	public SplineSegment(double initCurve, double initPercentage) {
		this.initCurve = initCurve;
		this.initPercentage = initPercentage;
	}
}
