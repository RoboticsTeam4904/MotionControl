package io.getcoffee.motionprofiles;


public class CubicSplineGenerator extends SplineGenerator {
	private final double initPosX, initPosY, finPosX, finPosY, initVelX, initVelY, finVelX, finVelY;
	protected final double[] PosX = new double[4], PosY = new double[4];
	protected final double[] VelX = new double[3], VelY = new double[3];
	protected final double[] AccX = new double[2], AccY = new double[2];
	protected final double[] JerkX = new double[1], JerkY = new double[1];

	public CubicSplineGenerator(double initPosX, double initPosY, double finPosX, double finPosY,
		double initVelX, double initVelY, double finVelX, double finVelY) {
		this.initPosX = initPosX;
		this.initPosY = initPosY;
		this.finPosX = finPosX;
		this.finPosY = finPosY;
		this.initVelX = initVelX;
		this.initVelY = initVelY;
		this.finVelX = finVelX;
		this.finVelY = finVelY;
		initializePos();
		initializeVel();
		initializeAcc();
		initializeJerk();
	}

	@Override
	/**
	 * Position
	 */
	protected void initializePos() {
		/* a */PosX[0] = 2 * initPosX - 2 * finPosX + initVelX + finVelX;
		/* b */PosX[1] = -3 * initPosX + 3 * finPosX - 2 * initVelX - finVelX;
		/* c */PosX[2] = initVelX;
		/* d */PosX[3] = initPosX;
		/* a */PosY[0] = 2 * initPosY - 2 * finPosY + initVelY + finVelY;
		/* b */PosY[1] = -3 * initPosY + 3 * finPosY - 2 * initVelY - finVelY;
		/* c */PosY[2] = initVelY;
		/* d */PosY[3] = initPosY;
	}

	@Override
	protected double PosX(double s) {
		return PosX[0] * s * s * s
			+ PosX[1] * s * s
			+ PosX[2] * s
			+ PosX[3];
	}

	@Override
	protected double PosY(double s) {
		return PosY[0] * s * s * s
			+ PosY[1] * s * s
			+ PosY[2] * s
			+ PosY[3];
	}

	@Override
	protected void initializeVel() {
		VelX[0] = PosX[0] * 3;
		VelX[1] = PosX[1] * 2;
		VelX[2] = PosX[2];
		VelY[0] = PosY[0] * 3;
		VelY[1] = PosY[1] * 2;
		VelY[2] = PosY[2];
	}

	@Override
	protected double VelX(double s) {
		return VelX[0] * s * s
			+ VelX[1] * s
			+ VelX[2];
	}

	@Override
	protected double VelY(double s) {
		return VelY[0] * s * s
			+ VelY[1] * s
			+ VelY[2];
	}

	@Override
	protected void initializeAcc() {
		AccX[0] = VelX[0] * 2;
		AccX[1] = VelX[1];
		AccY[0] = VelY[0] * 2;
		AccY[1] = VelY[1];
	}

	@Override
	protected double AccX(double s) {
		return AccX[0] * s
			+ AccX[1];
	}

	@Override
	protected double AccY(double s) {
		return AccY[0] * s
			+ AccY[1];
	}

	@Override
	protected void initializeJerk() {
		JerkX[0] = AccX[0];
		JerkY[0] = AccX[0];
	}

	@Override
	protected double JerkX(double s) {
		return JerkX[0];
	}

	@Override
	protected double JerkY(double s) {
		return JerkY[0];
	}
}
