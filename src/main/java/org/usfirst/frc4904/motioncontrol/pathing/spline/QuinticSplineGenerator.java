package org.usfirst.frc4904.motioncontrol.pathing.spline;

strictfp public class QuinticSplineGenerator extends SplineGenerator {
	private final double initPosX, initPosY, finPosX, finPosY, initVelX, initVelY, finVelX, finVelY, initAccX, initAccY, finAccX, finAccY;

	public QuinticSplineGenerator(double initPosX, double initPosY, double finPosX, double finPosY, double initVelX,
			double initVelY, double finVelX, double finVelY, double initAccX, double initAccY, double finAccX,
			double finAccY) {
		this.initPosX = initPosX;
		this.initPosY = initPosY;
		this.finPosX = finPosX;
		this.finPosY = finPosY;
		this.initVelX = initVelX;
		this.initVelY = initVelY;
		this.finVelX = finVelX;
		this.finVelY = finVelY;
		this.initAccX = initAccX;
		this.initAccY = initAccY;
		this.finAccX = finAccX;
		this.finAccY = finAccY;
		super.initialize();
	}

	@Override
	/**
	 * Solve for position polynomials
	 */
	protected void initializePos() {
		double ax = -6 * initPosX + 6 * finPosX - 3 * initVelX - 3 * finVelX - (initAccX / 2.0) + (finAccX / 2.0);
		double bx = 15 * initPosX - 15 * finPosX + 8 * initVelX + 7 * finVelX + (3.0 / 2.0) * initAccX - finAccX;
		double cx = -10 * initPosX + 10 * finPosX - 6 * initVelX - 4 * finVelX - (3.0 / 2.0) * initAccX
				+ (finAccX / 2.0);
		double dx = (initAccX / 2.0);
		double ex = initVelX;
		double fx = initPosX;
		double ay = -6 * initPosY + 6 * finPosY - 3 * initVelY - 3 * finVelY - (initAccY / 2.0) + (finAccY / 2.0);
		double by = 15 * initPosY - 15 * finPosY + 8 * initVelY + 7 * finVelY + (3.0 / 2.0) * initAccY - finAccY;
		double cy = -10 * initPosY + 10 * finPosY - 6 * initVelY - 4 * finVelY - (3.0 / 2.0) * initAccY
				+ (finAccY / 2.0);
		double dy = (initAccY / 2.0);
		double ey = initVelY;
		double fy = initPosY;
		PosX = new Polynomial(fx, ex, dx, cx, bx, ax);
		PosY = new Polynomial(fy, ey, dy, cy, by, ay);
	}
}