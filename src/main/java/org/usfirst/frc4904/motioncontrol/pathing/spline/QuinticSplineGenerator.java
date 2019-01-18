package org.usfirst.frc4904.motioncontrol.pathing.spline;

strictfp public class QuinticSplineGenerator extends SplineGenerator {
	private final double initPosX, initPosY, finPosX, finPosY, initVelX, initVelY, finVelX, finVelY, initAccX, initAccY,
			finAccX, finAccY;

	public QuinticSplineGenerator(double initPosX, double initPosY, double finPosX, double finPosY, double initVelX,
			double initVelY, double finVelX, double finVelY, double initAccX, double initAccY, double finAccX,
			double finAccY, double curveDerivativeThreshold, double granularity) {
		PosX = new double[6];
		PosY = new double[6];
		VelX = new double[5];
		VelY = new double[5];
		AccX = new double[4];
		AccY = new double[4];
		JerkX = new double[3];
		JerkY = new double[3];
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
		super.initialize(curveDerivativeThreshold, granularity);
	}

	public QuinticSplineGenerator(double initPosX, double initPosY, double finPosX, double finPosY, double initVelX,
			double initVelY, double finVelX, double finVelY, double initAccX, double initAccY, double finAccX,
			double finAccY, double curveDerivativeThreshold) {
		PosX = new double[6];
		PosY = new double[6];
		VelX = new double[5];
		VelY = new double[5];
		AccX = new double[4];
		AccY = new double[4];
		JerkX = new double[3];
		JerkY = new double[3];
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
		super.initialize(curveDerivativeThreshold);
	}

	public QuinticSplineGenerator(double initPosX, double initPosY, double finPosX, double finPosY, double initVelX,
			double initVelY, double finVelX, double finVelY, double initAccX, double initAccY, double finAccX,
			double finAccY) {
		PosX = new double[6];
		PosY = new double[6];
		VelX = new double[5];
		VelY = new double[5];
		AccX = new double[4];
		AccY = new double[4];
		JerkX = new double[3];
		JerkY = new double[3];
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
	 * Position
	 */
	protected void initializePos() {
		/* a */PosX[5] = -6 * initPosX + 6 * finPosX - 3 * initVelX - 3 * finVelX - (initAccX / 2.0) + (finAccX / 2.0);
		/* b */PosX[4] = 15 * initPosX - 15 * finPosX + 8 * initVelX + 7 * finVelX + (3.0 / 2.0) * initAccX - finAccX;
		/* c */PosX[3] = -10 * initPosX + 10 * finPosX - 6 * initVelX - 4 * finVelX - (3.0 / 2.0) * initAccX
				+ (finAccX / 2.0);
		/* d */PosX[2] = (initAccX / 2.0);
		/* e */PosX[1] = initVelX;
		/* f */PosX[0] = initPosX;
		/* a */PosY[5] = -6 * initPosY + 6 * finPosY - 3 * initVelY - 3 * finVelY - (initAccY / 2.0) + (finAccY / 2.0);
		/* b */PosY[4] = 15 * initPosY - 15 * finPosY + 8 * initVelY + 7 * finVelY + (3.0 / 2.0) * initAccY - finAccY;
		/* c */PosY[3] = -10 * initPosY + 10 * finPosY - 6 * initVelY - 4 * finVelY - (3.0 / 2.0) * initAccY
				+ (finAccY / 2.0);
		/* d */PosY[2] = (initAccY / 2.0);
		/* e */PosY[1] = initVelY;
		/* f */PosY[0] = initPosY;

	}
}