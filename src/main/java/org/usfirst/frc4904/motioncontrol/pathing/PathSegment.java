package org.usfirst.frc4904.motioncontrol.pathing;


import java.util.Map;
import java.util.TreeMap;

public class PathSegment {
	public TreeMap<Double, Double> lengthMap = new TreeMap<>();
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
					   double length, TreeMap<Double, Double> lengthMap) {
		this.initCurve = initCurve;
		this.finCurve = finCurve;
		this.maxSpeed = maxSpeed;
		this.maxCurve = maxCurve;
		this.maxCurveDerivative = maxCurveDerivative;
		this.length = length;
		this.lengthMap = lengthMap;
	}

	public PathSegment(double initCurve, double finCurve, double maxCurve, double maxCurveDerivative, double maxSpeed,
					   double minAcc, double maxAcc, double length, TreeMap<Double, Double> lengthMap) {
		this.initCurve = initCurve;
		this.finCurve = finCurve;
		this.maxSpeed = maxSpeed;
		this.maxCurve = maxCurve;
		this.maxCurveDerivative = maxCurveDerivative;
		this.length = length;
		this.lengthMap = lengthMap;
		System.out.println(lengthMap);
		this.minAcc = minAcc;
		this.maxAcc = maxAcc;
	}

	public PathSegment(double finCurve) {
		this.finCurve = finCurve;
	}

	public double nearestPercentage(double distance) {
		Map.Entry<Double, Double> lowEntry = lengthMap.floorEntry(distance);
		Map.Entry<Double, Double> highEntry = lengthMap.ceilingEntry(distance);
		if (lowEntry == null || highEntry == null) {
			return (lowEntry != null ? lowEntry.getValue() : highEntry.getValue());
		}
		if (Math.abs(distance - lowEntry.getKey()) < Math.abs(distance - highEntry.getKey())) {
			return lowEntry.getValue();
		}
		return highEntry.getValue();
	}

	public double extrapolatePercentage(double distance) {
		Map.Entry<Double, Double> lowEntry = lengthMap.floorEntry(distance);
		Map.Entry<Double, Double> highEntry = lengthMap.ceilingEntry(distance);
		if (lowEntry == null || highEntry == null) {
			return (lowEntry != null ? lowEntry.getValue() : highEntry.getValue());
		}
		double fraction = (distance - lowEntry.getKey())/(highEntry.getKey() - lowEntry.getKey());
		double out = lowEntry.getValue() + fraction * (highEntry.getValue() - lowEntry.getValue()) / (highEntry.getKey() - lowEntry.getKey());
		return out;
	}

	@Override
	public String toString() {
		return "PathSegment#{InitCurve: " + initCurve + ", FinCurve: " + finCurve + ", MaxCurve: " + maxCurve
			+ ", MaxCurveDeriv: " + maxCurveDerivative + ", Length: " + length + "}";
	}
}
