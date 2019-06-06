package org.usfirst.frc4904.motioncontrol.pathing;


import org.usfirst.frc4904.motioncontrol.pathing.PathGenerator;

strictfp public class SplineGenerator extends PathGenerator {
	protected Polynomial PosX, PosY, VelX, VelY, AccX, AccY, JerkX, JerkY;

	/**
	 * Solve for cubic position polynomials given intial and final constraints.
	 */
	public static SplineGenerator fitCubic(double initPosX, double initPosY, double finPosX, double finPosY,
		double initVelX, double initVelY, double finVelX, double finVelY) {
		double ax = 2 * initPosX - 2 * finPosX + initVelX + finVelX;
		double bx = -3 * initPosX + 3 * finPosX - 2 * initVelX - finVelX;
		double cx = initVelX;
		double dx = initPosX;
		double ay = 2 * initPosY - 2 * finPosY + initVelY + finVelY;
		double by = -3 * initPosY + 3 * finPosY - 2 * initVelY - finVelY;
		double cy = initVelY;
		double dy = initPosY;
		Polynomial PosX = new Polynomial(dx, cx, bx, ax);
		Polynomial PosY = new Polynomial(dy, cy, by, ay);
		return new SplineGenerator(PosX, PosY);
	}

	/**
	 * Solve for quintic position polynomials given intial and final constraints.
	 */
	public static SplineGenerator fitQuintic(double initPosX, double initPosY, double finPosX, double finPosY, double initVelX,
		double initVelY, double finVelX, double finVelY, double initAccX, double initAccY, double finAccX, double finAccY) {
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
		System.out.println("Posx " + fx + ", " + ex + ", " + dx + ", " + cx + ", " + bx + ", " + ax);
		System.out.println("Posy " + fy + ", " + ey + ", " + dy + ", " + cy + ", " + by + ", " + ay);
		Polynomial PosX = new Polynomial(fx, ex, dx, cx, bx, ax);
		Polynomial PosY = new Polynomial(fy, ey, dy, cy, by, ay);
		return new SplineGenerator(PosX, PosY);
	}

	public SplineGenerator(Polynomial PosX, Polynomial PosY) {
		this.PosX = PosX;
		this.PosY = PosY;
		makeDerivatives();
	}

	/**
	 * Construct the polynomials for derivatives of position.
	 */
	private void makeDerivatives() {
		VelX = PosX.derivative();
		VelY = PosY.derivative();
		AccX = VelX.derivative();
		AccY = VelY.derivative();
		JerkX = AccX.derivative();
		JerkY = AccY.derivative();
	}

	protected double PosX(double s) {
		return PosX.evaluate(s);
	}

	protected double PosY(double s) {
		return PosY.evaluate(s);
	}

	protected double VelX(double s) {
		return VelX.evaluate(s);
	}

	protected double VelY(double s) {
		return VelY.evaluate(s);
	}

	protected double AccX(double s) {
		return AccX.evaluate(s);
	}

	protected double AccY(double s) {
		return AccY.evaluate(s);
	}

	protected double JerkX(double s) {
		return JerkX.evaluate(s);
	}

	protected double JerkY(double s) {
		return JerkY.evaluate(s);
	}

	public static class Polynomial {
		double[] coefs;
		int length;

		public Polynomial(double... coefs) {
			this.coefs = coefs;
			this.length = coefs.length;
		}

		/**
		 * Calculate the derivative of a polynomial function using the power rule on
		 * each of its coefficients
		 *
		 * @return
		 */
		public Polynomial derivative() {
			if (length == 1) {
				return new Polynomial(0.0);
			}
			double[] dCoefs = new double[length - 1];
			for (int i = 1; i < length; i++) {
				dCoefs[i - 1] = coefs[i] * i;
			}
			return new Polynomial(dCoefs);
		}

		/**
		 * Evaluate the polynomial function at s
		 *
		 * @param s
		 *          the position along the spline from [0-1]
		 * @return
		 */
		protected double evaluate(double s) {
			double out = 0;
			for (int i = 0; i < coefs.length; i++) {
				out += coefs[i] * Math.pow(s, i);
			}
			return out;
		}
	}
}