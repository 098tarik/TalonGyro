package org.usfirst.frc.team86.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;

public class TankGyro {
	
	private TalonSRX right1;
	private TalonSRX left2;
	private TalonSRX left1;
	private NavX TalonGyro;
	private TalonSRX right2;
		
	private double percentRotation;
	private double gyroAngle;
	private double targetAngle;
	
//	private double arcLength;
//	private double leftTanVel;
//	private double rightTanVel;
//	private double arcAngle = Math.PI/2;
//	private double arcRadiusL;
//	
//	private double arcRadiusR = arcRadiusL + 25.5; 

//	public void calculateArcVel(double arcRadiusL,double arcRadiusR, double arcLength) {
//		arcLength = arcAngle*arcRadiusL;
//			}
	
	
	private double result;
	private states state = states.readGyro;
	
	private enum states {readGyro,  resetGyro, rotateToGyro}
	
    // PID constants from mecanumcode
	// PID constants
	private double kP = 1/180; // Proportional constant
	private double kI = 0.0; // Integral constant
	private double kD = 0.0; // Derivative constant
	private double kF = 0.0; // Feed Forward constant
	
	// PID variables
//	private double prevError = 0.0; // The error from the previous loop
//	private double integral = 0.0; // Error integrated over time
//	
//	private long prevTime;
//	
//	private double setpoint; // The target orientation for the robot
//	
//	private double tolerance = 1;
//	
//	private double maxOutput = 1.0;
//	private double minOutput = -1.0;
	
//	public void init(double setpoint, double p, double i, double d, double f) {
//		this.setpoint = setpoint;
//		this.kP = p;
//		this.kI = i;
//		this.kD = d;
//		this.kF = f;
//		this.prevError = 0.0;
//		this.integral = 0.0;
//	}	
	public TankGyro(NavX TalonGyro, TalonSRX left1, TalonSRX left2, TalonSRX right1, TalonSRX right2) {
		this.TalonGyro = TalonGyro;
		this.left1 = left1;
		this.left2 = left2;
		this.right1 = right1;
		this.right2 = right2;
		
		left2.set(ControlMode.Follower, left1.getDeviceID());
        right2.set(ControlMode.Follower, right1.getDeviceID());
    
		left1.set(ControlMode.PercentOutput,0);
	    right1.set(ControlMode.PercentOutput, 0);
		
	}

	// Change Gyro State (Read Gyro, Reset Gyro, Turn to Gyro Angle)
	public void updateGyro() {
		gyroAngle = TalonGyro.getNormalizedAngle();
		state = states.rotateToGyro;	
		if(IO.resetButton.equals(true)) {
			state = state.resetGyro;
			}
		switch(state) {
		
		case readGyro:
			TalonGyro.getNormalizedAngle();
			if(IO.rotateButton.equals(true)) {
				state = state.rotateToGyro;
			}
			break;
			
		case resetGyro:
			TalonGyro.reset();
			state = states.readGyro;
			break;
			
		case rotateToGyro:		   
		   // Error of gyro angle to target angle
			targetAngle = 90;
			gyroAngle = TalonGyro.getNormalizedAngle();

		   double error = (targetAngle - gyroAngle)*1/180;
		
//		   // angle correction for shortest 
//		   if (Math.abs(error) > 180) { 
//				if (error > 0) { 
//					error = error - 360;
//				} else { 
//				    error =  error + 360;
//				}
//				
//				//PID  
//		    double result = (kP * error) + (kI * integral) + (kD * (error - prevError));
//		        if (result > 0) {
//		        	result += kF;
//		        } else {
//		        	result -= kF;
//		        }
//		       	prevError = error;
//		       	
//		        if (result > 1) {
//		          result = 1;
//		        } else if (result < -1) {
//		          result = -1;
//		        }
//			
		   
		   // percent rotation to pass through percent output
		      percentRotation = result;
		   
		      left1.set(ControlMode.PercentOutput, percentRotation);
		      right1.set(ControlMode.PercentOutput, -percentRotation);
		    
		      break;
		}
	}
	public void turnToAngle(double targetAngle) {
		this.targetAngle = targetAngle;
	}
}
