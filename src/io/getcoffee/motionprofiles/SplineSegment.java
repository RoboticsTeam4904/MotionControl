package io.getcoffee.motionprofiles;


import java.util.Map;
import java.util.TreeMap;

public class SplineSegment {
	public TreeMap<Double, SplinePoint> lengthMap = new TreeMap<>();
	protected double initCurve;
	protected double finCurve;
	protected double maxSpeed;
	protected double maxCurve;
	protected double maxCurveDerivative;
	protected double finPercentage;
	protected double length;

	public SplineSegment(double initCurve, double finCurve, double maxCurve, double maxCurveDerivative, double maxSpeed,
						 double length, TreeMap<Double, SplinePoint> lengthMap) {
		this.initCurve = initCurve;
		this.finCurve = finCurve;
		this.maxSpeed = maxSpeed;
		this.maxCurve = maxCurve;
		this.maxCurveDerivative = maxCurveDerivative;
		this.length = length;
		this.lengthMap = lengthMap;
	}

	public SplineSegment(double initCurve) {
		this.initCurve = initCurve;
	}

	public SplinePoint findNearestPoint(double distance) {
		Map.Entry<Double, SplinePoint> lowPoint = lengthMap.floorEntry(distance);
		Map.Entry<Double, SplinePoint> highPoint = lengthMap.ceilingEntry(distance);
		if (lowPoint == null || highPoint == null) {
			return (lowPoint != null ? lowPoint.getValue() : highPoint.getValue());
		}
		if (Math.abs(distance - lowPoint.getKey()) < Math.abs(distance - highPoint.getKey())) {
			return lowPoint.getValue();
		}
		return highPoint.getValue();
	}

	@Override
	public String toString() {
		return "SplineSegment#{InitCurve: " + initCurve + ", FinCurve: " + finCurve + ", MaxCurve: " + maxCurve
			+ ", MaxCurveDeriv: " + maxCurveDerivative + ", Length: " + length + "}";
	}
}
