package org.usfirst.frc4904.motioncontrol.pathing.spline;

strictfp public class CubicSplineGenerator extends SplineGenerator {
	private final double initPosX, initPosY, finPosX, finPosY, initVelX, initVelY, finVelX, finVelY;

	public CubicSplineGenerator(double initPosX, double initPosY, double finPosX, double finPosY, double initVelX,
			double initVelY, double finVelX, double finVelY) {
		this.initPosX = initPosX;
		this.initPosY = initPosY;
		this.finPosX = finPosX;
		this.finPosY = finPosY;
		this.initVelX = initVelX;
		this.initVelY = initVelY;
		this.finVelX = finVelX;
		this.finVelY = finVelY;
		super.initialize();
	}

	@Override
	/**
	 * Solve for position polynomials
	 */
	protected void initializePos() {
		double ax = 2 * initPosX - 2 * finPosX + initVelX + finVelX;
		double bx = -3 * initPosX + 3 * finPosX - 2 * initVelX - finVelX;
		double cx = initVelX;
		double dx = initPosX;
		double ay = 2 * initPosY - 2 * finPosY + initVelY + finVelY;
		double by = -3 * initPosY + 3 * finPosY - 2 * initVelY - finVelY;
		double cy = initVelY;
		double dy = initPosY;
		PosX = new Polynomial(dx, cx, bx, ax);
		PosY = new Polynomial(dy, cy, by, ay);
	}
}