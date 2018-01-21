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
	private double angle;

	private states state = states.readGyro;
	
	private enum states {readGyro,  resetGyro, rotateToGyro}
	
    // PID constants from mecanumcode
	// PID constants
	private double kP = 0.0; // Proportional constant
	private double kI = 0.0; // Integral constant
	private double kD = 0.0; // Derivative constant
	private double kF = 0.0; // Feed Forward constant
	
	// PID variables
	private double prevError = 0.0; // The error from the previous loop
	private double integral = 0.0; // Error integrated over time
	
	private long prevTime;
	
	private double setpoint; // The target orientation for the robot
	
	private double tolerance = 1;
	
	private double maxOutput = 1.0;
	private double minOutput = -1.0;
	
	public void init(double setpoint, double p, double i, double d, double f) {
		this.setpoint = setpoint;
		this.kP = p;
		this.kI = i;
		this.kD = d;
		this.kF = f;
		this.prevError = 0.0;
		this.integral = 0.0;
	}
	
	
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
	public void gyroState() {
		
		if(IO.rotateButton.equals(true)) {
			turnToAngle(90);
			state = states.rotateToGyro;
		} else {
			state = states.readGyro;
		}

		switch(state) {
		
		case readGyro:
			TalonGyro.getNormalizedAngle();
			break;
			
		case resetGyro:
			TalonGyro.reset();
			targetAngle = 0;
			state = states.readGyro;
			break;
			
		case rotateToGyro:
		   TalonGyro.getNormalizedAngle();
		   
		   kP = 1/360;
		    // Error of gyro angle to target angle
		   double preverror = targetAngle - gyroAngle;
		   double error = preverror*kP;
		   
		   // angle correction for shortest path from Mecanum
		   if (Math.abs(error) > 180) { // if going around the other way is closer
				if (error > 0) { // if positive
					error = error - 360;
				} else { // if negative
				    error =  error + 360;
				}
				
				//PID from mecanum //TODO look at this at home and finish + test Monday Night
				double maxI = 0.4;
				if (kI != 0) {
		            double potentialIGain = (integral + error) * kI;
		            if (potentialIGain < maxI) {
		              if (potentialIGain > -maxI) {
		                integral += error;
		              } else {
		                integral = -maxI; // -1 / kI
		              }
		            } else {
		              integral = maxI; // 1 / kI
		            }
		        } else {
		        	integral = 0;
		        }
				
				if (Math.abs(error) < 3.0) {
					error = 0;
				}
				
		    double result = (kP * error) + (kI * integral) + (kD * (error - prevError));
		        if (result > 0) {
		        	result += kF;
		        } else {
		        	result -= kF;
		        }
		       	prevError = error;
		       	
		        if (result > 1) {
		          result = 1;
		        } else if (result < -1) {
		          result = -1;
		        }
			}
		   
		   // percent rotation to pass through percent output
		   // as error approaches zero, percent output approaches zero (i.e target reached)
		      percentRotation = error/targetAngle;
		   
		      left1.set(ControlMode.PercentOutput, percentRotation);
		      right1.set(ControlMode.PercentOutput, percentRotation);
		    
		      state = states.resetGyro;
			break;
		}
	
		
	}
		
	public void turnToAngle(double angle) {
		this.angle = targetAngle;
	}

}
