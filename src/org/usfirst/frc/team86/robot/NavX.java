package org.usfirst.frc.team86.robot;

import com.kauailabs.navx.frc.AHRS;

public class NavX {
	
	public AHRS ahrs;

	public double calibrate() {
		return ahrs.calibrate(); // calibrate gyro
	}
	
	public double getNormalizedAngle() {
		return ((ahrs.getAngle() % 360) + 360) % 360;
	}

	public double getRawAngle() {
		return ahrs.getAngle();
	}

	public void reset() {
		ahrs.reset();
	}

	public AHRS getAHRS() {
		return ahrs;
	}

}
