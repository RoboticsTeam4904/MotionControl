package io.getcoffee.motionprofiles;

/**
 * Created by howard on 3/13/17.
 */
public enum WheelSide {
	LEFT(-1), RIGHT(1);

	private int modifier;
	WheelSide(int modifier) {
		this.modifier = modifier;
	}

	public int getModfier() {
		return this.modifier;
	}

}
