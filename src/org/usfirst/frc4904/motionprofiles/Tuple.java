package org.usfirst.frc4904.motionprofiles;


public class Tuple<X, Y> {
	private final X x;
	private final Y y;

	public Tuple(X x, Y y) {
		this.x = x;
		this.y = y;
	}

	public X getX() {
		return x;
	}

	public Y getY() {
		return y;
	}
	
	@Override
	public String toString() {
		return "Tuple[" + x + ", " + y + "]";
	}
}
