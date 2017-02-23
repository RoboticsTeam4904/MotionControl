package io.getcoffee.motionprofiles;


public class SplineSegment {
	protected double initCurve;
	protected double finCurve;
	protected double maxCurve;
	protected double maxCurveDerivative;
	public double initPercentage;
	protected double finPercentage;
	protected double length;

	public SplineSegment(double initCurve, double finCurve, double maxCurve, double maxCurveDerivative, double length) {
		this.initCurve = initCurve;
		this.finCurve = finCurve;
		this.maxCurve = maxCurve;
		this.maxCurveDerivative = maxCurveDerivative;
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
