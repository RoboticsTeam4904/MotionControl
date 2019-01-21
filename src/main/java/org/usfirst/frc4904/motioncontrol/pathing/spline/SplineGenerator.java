
package org.usfirst.frc4904.motioncontrol.pathing.spline;

import org.usfirst.frc4904.motioncontrol.pathing.PathGenerator;

strictfp public abstract class SplineGenerator extends PathGenerator {
	protected Polynomial PosX, PosY, VelX, VelY, AccX, AccY, JerkX, JerkY;

	protected void initialize() {
		initializePos();
		initializeDerivatives();
		super.initialize();
	}

	protected void initialize(double curveDerivativeThreshold) {
		initializePos();
		initializeDerivatives();
		super.initialize(curveDerivativeThreshold);
	}

	// TODO: gets called by super initialize(threshold)
	public void initialize(double curveDerivativeThreshold, double granularity) {
		initializePos();
		initializeDerivatives();
		super.initialize(curveDerivativeThreshold, granularity);
	}


	/**
	 * Initialize the polynomial coefficients for derivatives of position
	 */
	protected void initializeDerivatives() {
		VelX = PosX.derivative();
		VelY = PosY.derivative();
		AccX = VelX.derivative();
		AccY = VelY.derivative();
		JerkX = AccX.derivative();
		JerkY = AccY.derivative();
	}

	/**
	 * Initialize the position polynomial coefficients
	 */
	protected abstract void initializePos();

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

	protected class Polynomial {
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
		 * Evaluate a polynomial function at s
		 *
		 * @param s
		 *            the position along the spline from [0-1]
		 * @return
		 */
		protected double evaluate(double s) {
			double out = 0;

			for (int i = 0; i < coefs.length; i++) {// coefs.length - 1; i >= 0;
				// System.out.println(Double.toString(i) + " power, coef " +
				// Double.toString(coefs[i])); // i--) {
				out += coefs[i] * Math.pow(s, i);
			}
			return out;
		}
	}
}