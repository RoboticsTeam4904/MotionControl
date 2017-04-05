package io.getcoffee.motionprofiles.pathing;


import java.util.Map;
import java.util.TreeMap;

public class PathSegment {
	public TreeMap<Double, PathPoint> lengthMap = new TreeMap<>();
	public double initCurve;
	public double finCurve;
	public double maxSpeed;
	public double maxCurve;
	public double maxCurveDerivative;
	public double minAcc;
	public double maxAcc;
	public double finPercentage;
	public double length;

	public PathSegment(double initCurve, double finCurve, double maxCurve, double maxCurveDerivative, double maxSpeed,
					   double length, TreeMap<Double, PathPoint> lengthMap) {
		this.initCurve = initCurve;
		this.finCurve = finCurve;
		this.maxSpeed = maxSpeed;
		this.maxCurve = maxCurve;
		this.maxCurveDerivative = maxCurveDerivative;
		this.length = length;
		this.lengthMap = lengthMap;
	}

	public PathSegment(double initCurve, double finCurve, double maxCurve, double maxCurveDerivative, double maxSpeed,
					   double minAcc, double maxAcc, double length, TreeMap<Double, PathPoint> lengthMap) {
		this.initCurve = initCurve;
		this.finCurve = finCurve;
		this.maxSpeed = maxSpeed;
		this.maxCurve = maxCurve;
		this.maxCurveDerivative = maxCurveDerivative;
		this.length = length;
		this.lengthMap = lengthMap;
		this.minAcc = minAcc;
		this.maxAcc = maxAcc;
	}

	public PathSegment(double finCurve) {
		this.finCurve = finCurve;
	}

	public PathPoint findNearestPoint(double distance) {
		Map.Entry<Double, PathPoint> lowPoint = lengthMap.floorEntry(distance);
		Map.Entry<Double, PathPoint> highPoint = lengthMap.ceilingEntry(distance);
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
		return "PathSegment#{InitCurve: " + initCurve + ", FinCurve: " + finCurve + ", MaxCurve: " + maxCurve
			+ ", MaxCurveDeriv: " + maxCurveDerivative + ", Length: " + length + "}";
	}
}
