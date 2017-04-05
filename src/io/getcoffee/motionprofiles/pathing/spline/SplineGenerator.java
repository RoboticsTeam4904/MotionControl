package io.getcoffee.motionprofiles.pathing.spline;


import io.getcoffee.motionprofiles.pathing.PathGenerator;

strictfp public abstract class SplineGenerator extends PathGenerator {

	/**
	 * Initialize the position polynomial coefficients
	 */
	protected abstract void initializePos();

	/**
	 * Initialize the velocity polynomial coefficients. The complexity for this is much
	 * simpler since you can use the position coefficients.
	 */
	protected abstract void initializeVel();

	/**
	 * Initialize the acceleration polynomial coefficients.
	 */
	protected abstract void initializeAcc();

	/**
	 * Initialize the jerk polynomial coefficients. For a cubic spline this should be the simplest.
	 */
	protected abstract void initializeJerk();
}