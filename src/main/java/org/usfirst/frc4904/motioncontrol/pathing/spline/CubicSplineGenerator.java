package org.usfirst.frc4904.motioncontrol.pathing.spline;

strictfp public class CubicSplineGenerator extends SplineGenerator {
	private final double initPosX, initPosY, finPosX, finPosY, initVelX, initVelY, finVelX, finVelY;

	public CubicSplineGenerator(double initPosX, double initPosY, double finPosX, double finPosY, double initVelX,
			double initVelY, double finVelX, double finVelY, double curveDerivativeThreshold, double granularity) {
		PosX = new double[4];
		PosY = new double[4];
		VelX = new double[3];
		VelY = new double[3];
		AccX = new double[2];
		AccY = new double[2];
		JerkX = new double[1];
		JerkY = new double[1];

		this.initPosX = initPosX;
		this.initPosY = initPosY;
		this.finPosX = finPosX;
		this.finPosY = finPosY;
		this.initVelX = initVelX;
		this.initVelY = initVelY;
		this.finVelX = finVelX;
		this.finVelY = finVelY;
		super.initialize(curveDerivativeThreshold, granularity);
	}

	public CubicSplineGenerator(double initPosX, double initPosY, double finPosX, double finPosY, double initVelX,
			double initVelY, double finVelX, double finVelY, double curveDerivativeThreshold) {
		PosX = new double[4];
		PosY = new double[4];
		VelX = new double[3];
		VelY = new double[3];
		AccX = new double[2];
		AccY = new double[2];
		JerkX = new double[1];
		JerkY = new double[1];

		this.initPosX = initPosX;
		this.initPosY = initPosY;
		this.finPosX = finPosX;
		this.finPosY = finPosY;
		this.initVelX = initVelX;
		this.initVelY = initVelY;
		this.finVelX = finVelX;
		this.finVelY = finVelY;
		super.initialize(curveDerivativeThreshold);
	}

	public CubicSplineGenerator(double initPosX, double initPosY, double finPosX, double finPosY, double initVelX,
			double initVelY, double finVelX, double finVelY) {
		PosX = new double[4];
		PosY = new double[4];
		VelX = new double[3];
		VelY = new double[3];
		AccX = new double[2];
		AccY = new double[2];
		JerkX = new double[1];
		JerkY = new double[1];

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
	protected void initializePos() {
		/* a */PosX[3] = 2 * initPosX - 2 * finPosX + initVelX + finVelX;
		/* b */PosX[2] = -3 * initPosX + 3 * finPosX - 2 * initVelX - finVelX;
		/* c */PosX[1] = initVelX;
		/* d */PosX[0] = initPosX;
		/* a */PosY[3] = 2 * initPosY - 2 * finPosY + initVelY + finVelY;
		/* b */PosY[2] = -3 * initPosY + 3 * finPosY - 2 * initVelY - finVelY;
		/* c */PosY[1] = initVelY;
		/* d */PosY[0] = initPosY;
	}
}