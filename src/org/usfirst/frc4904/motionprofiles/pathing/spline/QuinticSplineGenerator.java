package org.usfirst.frc4904.motionprofiles.pathing.spline;


strictfp public class QuinticSplineGenerator extends SplineGenerator {
	private final double initPosX, initPosY, finPosX, finPosY, initVelX, initVelY, finVelX, finVelY, initAccX, initAccY,
		finAccX, finAccY;
	// TODO: Turn these into individual doubles to remove unnecessary overhead that comes with non-dynamic access of any of the variables.
	protected final double[] PosX = new double[6], PosY = new double[6];
	protected final double[] VelX = new double[5], VelY = new double[5];
	protected final double[] AccX = new double[4], AccY = new double[4];
	protected final double[] JerkX = new double[3], JerkY = new double[3];

	public QuinticSplineGenerator(double initPosX, double initPosY, double finPosX, double finPosY,
								  double initVelX, double initVelY, double finVelX, double finVelY,
								  double initAccX, double initAccY, double finAccX, double finAccY) {
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
		initialize(0.2);
	}

	@Override
	/**
	 * Position
	 */
	protected void initializePos() {
		/* a */PosX[0] = -6 * initPosX + 6 * finPosX - 3 * initVelX - 3 * finVelX - (initAccX / 2.0) + (finAccX / 2.0);
		/* b */PosX[1] = 15 * initPosX - 15 * finPosX + 8 * initVelX + 7 * finVelX + (3.0 / 2.0) * initAccX - finAccX;
		/* c */PosX[2] = -10 * initPosX + 10 * finPosX - 6 * initVelX - 4 * finVelX - (3.0 / 2.0) * initAccX + (finAccX / 2.0);
		/* d */PosX[3] = (initAccX / 2.0);
		/* e */PosX[4] = initVelX;
		/* f */PosX[5] = initPosX;
		/* a */PosY[0] = -6 * initPosY + 6 * finPosY - 3 * initVelY - 3 * finVelY - (initAccY / 2.0) + (finAccY / 2.0);
		/* b */PosY[1] = 15 * initPosY - 15 * finPosY + 8 * initVelY + 7 * finVelY + (3.0 / 2.0) * initAccY - finAccY;
		/* c */PosY[2] = -10 * initPosY + 10 * finPosY - 6 * initVelY - 4 * finVelY - (3.0 / 2.0) * initAccY + (finAccY / 2.0);
		/* d */PosY[3] = (initAccY / 2.0);
		/* e */PosY[4] = initVelY;
		/* f */PosY[5] = initPosY;
	}
}