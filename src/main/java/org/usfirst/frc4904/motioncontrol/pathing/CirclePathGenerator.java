package org.usfirst.frc4904.motioncontrol.pathing;

strictfp public class CirclePathGenerator extends PathGenerator {
	private final double initPosX, initPosY, finPosX, finPosY, finAngle;
	protected final double centerX, centerY, radius, initAngle;
	protected final double absoluteLength;

	public CirclePathGenerator(double initPosX, double initPosY, double finPosX, double finPosY, double finAngle) {
		finAngle = finAngle + Math.PI / 2; // Only works if we are turning left
		this.initPosX = initPosX;
		this.initPosY = initPosY;
		this.finPosX = finPosX;
		this.finPosY = finPosY;
		this.finAngle = finAngle;
		this.radius = calcRadius(initPosX, initPosY, finPosX, finPosY, finAngle);
		this.centerX = finPosX + radius * Math.cos(finAngle);
		this.centerY = finPosY + radius * Math.sin(finAngle);
		this.initAngle = Math.atan2(initPosY - centerY, initPosX - centerX);
		this.absoluteLength = 2 * (finAngle - initAngle) * radius;
		// System.out.println("Radius, " + Double.toString(this.radius) + "
		// CenterX, " + Double.toString(this.centerX)
		// + " CenterY, " + Double.toString(this.centerY));
		// System.out.println(Double.toString(this.initAngle) + " init, fin " +
		// Double.toString(this.finAngle));
		// double a = 0.6;
		// System.out.println("angle, " + Double.toString(percentageToAngle(a)) + " x,
		// "
		// + Double.toString(radius * Math.cos(percentageToAngle(a))) + " y, "
		// + Double.toString(radius * Math.sin(percentageToAngle(a))));
		segment();
	}

	protected double calcRadius(double initPosX, double initPosY, double finPosX, double finPosY, double phi) {
		return ((initPosX - finPosX) * (initPosX - finPosX) + (initPosY - finPosY) * (initPosY - finPosY))
				/ (2 * ((initPosX - finPosX) * Math.cos(phi) + (initPosY - finPosY) * Math.sin(phi)));
	}

	protected double percentageToAngle(double t, double initAngle, double finAngle) {
		return (finAngle - initAngle) * t + initAngle;
	}

	protected double percentageToAngle(double s) {
		return percentageToAngle(s, initAngle, finAngle);
	}

	protected double PosX(double s) {
		return centerX - radius * Math.cos(percentageToAngle(s));
	}

	protected double PosY(double s) {
		return centerY - radius * Math.sin(percentageToAngle(s));
	}

	protected double VelX(double s) {
		return radius * Math.sin(percentageToAngle(s)) * (finAngle - initAngle);
	}

	protected double VelY(double s) {
		return -radius * Math.cos(percentageToAngle(s)) * (finAngle - initAngle);
	}

	protected double AccX(double s) {
		return radius * Math.cos(percentageToAngle(s)) * (finAngle - initAngle) * (finAngle - initAngle);
	}

	protected double AccY(double s) {
		return radius * Math.sin(percentageToAngle(s)) * (finAngle - initAngle) * (finAngle - initAngle);
	}

	protected double JerkX(double s) {
		return -radius * Math.sin(percentageToAngle(s)) * (finAngle - initAngle) * (finAngle - initAngle)
				* (finAngle - initAngle);
	}

	protected double JerkY(double s) {
		return radius * Math.cos(percentageToAngle(s)) * (finAngle - initAngle) * (finAngle - initAngle)
				* (finAngle - initAngle);
	}
}