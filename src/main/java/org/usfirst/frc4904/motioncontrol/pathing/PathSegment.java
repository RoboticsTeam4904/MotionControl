package org.usfirst.frc4904.motioncontrol.pathing;


import java.util.Map;
import java.util.TreeMap;

public class PathSegment {
	public final TreeMap<Double, Double> lengthMap;
	public final double maxCurve;
	public final double length;
	public final double minAcc;
	public final double maxAcc;

	public PathSegment(double maxCurve, double minAcc, double maxAcc, double length, TreeMap<Double, Double> lengthMap) {
		this.maxCurve = maxCurve;
		this.length = length;
		this.lengthMap = lengthMap;
		this.minAcc = minAcc;
		this.maxAcc = maxAcc;
//		System.out.println(lengthMap);
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
		// System.out.println("low entry: " + lowEntry + ", " + "high entry: " + highEntry + ", L" +  (lowEntry == null) + ", H" +  (highEntry == null));
		if (lowEntry == null || highEntry == null) {
			return (lowEntry != null ? lowEntry.getValue() : highEntry.getValue());
		}
		if (highEntry.getKey() == lowEntry.getKey()) { // to prevent division by 0
			return lowEntry.getValue();
		}
		double fraction = (distance - lowEntry.getKey())/(highEntry.getKey() - lowEntry.getKey());
		double out = lowEntry.getValue() + fraction * (highEntry.getValue() - lowEntry.getValue());
		return out;
	}

	@Override
	public String toString() {
		return "PathSegment#{MaxCurve: " + maxCurve + ", Length: " + length + ", Acc: " + minAcc + " to " + maxAcc + "}";
	}
}
