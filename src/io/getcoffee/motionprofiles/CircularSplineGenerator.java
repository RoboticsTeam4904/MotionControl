package io.getcoffee.motionprofiles;


public abstract class CircularSplineGenerator extends SplineGenerator {
	private final double initPosX, initPosY, finPosX, finPosY, finAngle;
	protected final double centerX, centerY, radius, initAngle;
	protected final double absoluteLength;

	CircularSplineGenerator(double initPosX, double initPosY, double finPosX, double finPosY, double finAngle) {
		finAngle = finAngle - Math.PI / 2;
		this.initPosX = initPosX;
		this.initPosY = initPosY;
		this.finPosX = finPosX;
		this.finPosY = finPosY;
		this.finAngle = finAngle;
		this.radius = calcRadius(initPosX, initPosY, finPosX, finPosY, finAngle);
		this.centerX = finPosX + radius * Math.cos(finAngle);
		this.centerY = finPosY + radius * Math.sin(finAngle);
		this.initAngle = Math.atan2(initPosX - centerX, initPosY - centerY);
		// this.radius = Math.abs(radius)
		this.absoluteLength = 2 * (finAngle - initAngle) * radius;
	}

	protected double calcRadius(double initPosX, double initPosY, double finPosX, double finPosY, double phi) {
		return ((initPosX - finPosX) * (initPosX - finPosX) + (initPosY - finPosY) * (initPosY - finPosY))
			/ (2 * ((initPosX - finPosX) * Math.cos(phi) + (initPosY - finPosY) * Math.sin(phi)));
	}

	protected double timeToAngle(double t, double initAngle, double finAngle) {
		return (finAngle - initAngle) * t + initAngle;
	}

	protected double timeToAngle(double t) {
		return timeToAngle(t, initAngle, finAngle);
	}

	protected double PosX(double t) {
		return centerX + radius * Math.cos(timeToAngle(t));
	}

	protected double PosY(double t) {
		return centerY + radius * Math.sin(timeToAngle(t));
	}

	protected double VelX(double t) {
		return -radius * Math.sin(timeToAngle(t)) * (finAngle - initAngle);
	}

	protected double VelY(double t) {
		return radius * Math.cos(timeToAngle(t)) * (finAngle - initAngle);
	}

	protected double AccX(double t) {
		return -radius * Math.cos(timeToAngle(t)) * (finAngle - initAngle) * (finAngle - initAngle);
	}

	protected double AccY(double t) {
		return -radius * Math.sin(timeToAngle(t)) * (finAngle - initAngle) * (finAngle - initAngle);
	}

	protected double JerkX(double t) {
		return radius * Math.sin(timeToAngle(t)) * (finAngle - initAngle) * (finAngle - initAngle) * (finAngle - initAngle);
	}

	protected double JerkY(double t) {
		return -radius * Math.cos(timeToAngle(t)) * (finAngle - initAngle) * (finAngle - initAngle) * (finAngle - initAngle);
	}
}