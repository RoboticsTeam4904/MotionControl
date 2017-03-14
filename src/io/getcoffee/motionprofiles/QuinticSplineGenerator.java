package io.getcoffee.motionprofiles;


public class QuinticSplineGenerator extends SplineGenerator {
	private final double initPosX, initPosY, finPosX, finPosY, initVelX, initVelY, finVelX, finVelY, initAccX, initAccY,
		finAccX, finAccY;
	// TODO: Turn these into individual doubles to remove unnecessary overhead that comes with non-dynamic access of any of the variables.
	protected final double[] PosX = new double[6], PosY = new double[6];
	protected final double[] VelX = new double[5], VelY = new double[5];
	protected final double[] AccX = new double[4], AccY = new double[4];
	protected final double[] JerkX = new double[3], JerkY = new double[3];
	protected final double absoluteLength;

	QuinticSplineGenerator(double initPosX, double initPosY, double finPosX, double finPosY,
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
		initializePos();
		initializeVel();
		initializeAcc();
		absoluteLength = calcAbsoluteLength();
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

	@Override
	protected double PosX(double s) {
		return PosX[0] * s * s * s * s * s
			+ PosX[1] * s * s * s * s
			+ PosX[2] * s * s * s
			+ PosX[3] * s * s
			+ PosX[4] * s
			+ PosX[5];
	}

	@Override
	protected double PosY(double s) {
		return PosY[0] * s * s * s * s * s
			+ PosY[1] * s * s * s * s
			+ PosY[2] * s * s * s
			+ PosY[3] * s * s
			+ PosY[4] * s
			+ PosY[5];
	}

	@Override
	protected void initializeVel() {
		VelX[0] = PosX[0] * 5;
		VelX[1] = PosX[1] * 4;
		VelX[2] = PosX[2] * 3;
		VelX[3] = PosX[3] * 2;
		VelX[4] = PosX[4];
		VelY[0] = PosY[0] * 5;
		VelY[1] = PosY[1] * 4;
		VelY[2] = PosY[2] * 3;
		VelY[3] = PosY[3] * 2;
		VelY[4] = PosY[4];
	}

	@Override
	protected double VelX(double s) {
		return VelX[0] * s * s * s * s
			+ VelX[1] * s * s * s
			+ VelX[2] * s * s
			+ VelX[3] * s
			+ VelX[4];
	}

	@Override
	protected double VelY(double s) {
		return VelY[0] * s * s * s * s
			+ VelY[1] * s * s * s
			+ VelY[2] * s * s
			+ VelY[3] * s
			+ VelY[4];
	}

	@Override
	protected void initializeAcc() {
		AccX[0] = VelX[0] * 4;
		AccX[1] = VelX[1] * 3;
		AccX[2] = VelX[2] * 2;
		AccX[3] = VelX[3];
		AccY[0] = VelY[0] * 4;
		AccY[1] = VelY[1] * 3;
		AccY[2] = VelY[2] * 2;
		AccY[3] = VelY[3];
	}

	@Override
	protected double AccX(double s) {
		return AccX[0] * s * s * s
			+ AccX[1] * s * s
			+ AccX[2] * s
			+ AccX[3];
	}

	@Override
	protected double AccY(double s) {
		return AccY[0] * s * s * s
			+ AccY[1] * s * s
			+ AccY[2] * s
			+ AccY[3];
	}

	@Override
	protected void initializeJerk() {
		JerkX[0] = AccX[0] * 3;
		JerkX[1] = AccX[1] * 2;
		JerkX[2] = AccX[2];
		JerkY[0] = AccY[0] * 3;
		JerkY[1] = AccY[1] * 2;
		JerkY[2] = AccY[2];
	}

	@Override
	protected double JerkX(double s) {
		return JerkX[0] * s * s
				+ JerkX[1] * s
				+ JerkX[2];
	}

	@Override
	protected double JerkY(double s) {
		return JerkY[0] * s * s
				+ JerkX[1] * s
				+ JerkY[2];
	}
}
