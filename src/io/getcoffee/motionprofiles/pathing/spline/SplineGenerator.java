package io.getcoffee.motionprofiles.pathing.spline;


strictfp public abstract class SplineGenerator extends io.getcoffee.motionprofiles.pathing.PathGenerator {
	protected void initialize() {
		initializePos();
		initializeDerivatives();
		super.initialize();
	}

	/**
	 * Calculate the derivative of a polynomial function using the power rule on each of its coefficients
	 * 
	 * @param coefs
	 * @return
	 */
	protected double[] derivative(double[] coefs) {
		int length = coefs.length;
		double[] dCoefs = new double[length - 1];
		for (int i = 1; i < length; i++) {
			dCoefs[i - 1] = coefs[i] * i;
		}
		return dCoefs;
	}

	/**
	 * Evaluate a polynomial function at s
	 * 
	 * @param coefs
	 *        the coefficients of the polynomial
	 * @param s
	 *        the position along the spline from [0-1]
	 * @return
	 */
	protected double evaluate(double[] coefs, double s) {
		double out = 0;
		for (int i = 0; i < coefs.length; i++) {
			out += coefs[i] * Math.pow(s, i);
		}
		return out;
	}

	/**
	 * Initialize the polynomial coefficients for derivatives of position
	 */
	protected void initializeDerivatives() {
		VelX = derivative(PosX);
		VelY = derivative(PosY);
		AccX = derivative(VelX);
		AccY = derivative(VelY);
		JerkX = derivative(AccX);
		JerkY = derivative(AccY);
	}

	/**
	 * Initialize the position polynomial coefficients
	 */
	protected abstract void initializePos();

	protected double PosX(double s) {
		return evaluate(PosX, s);
	}

	protected double PosY(double s) {
		return evaluate(PosY, s);
	}

	protected double VelX(double s) {
		return evaluate(VelX, s);
	}

	protected double VelY(double s) {
		return evaluate(VelY, s);
	}

	protected double AccX(double s) {
		return evaluate(AccX, s);
	}

	protected double AccY(double s) {
		return evaluate(AccY, s);
	}

	protected double JerkX(double s) {
		return evaluate(JerkX, s);
	}

	protected double JerkY(double s) {
		return evaluate(JerkY, s);
	}
}