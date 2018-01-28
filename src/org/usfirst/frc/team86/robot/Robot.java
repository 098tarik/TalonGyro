package org.usfirst.frc.team86.robot;

import org.usfirst.frc.team86.robot.TalonDrive.Gear;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

    private TalonSRX left1;
    private TalonSRX left2;
    private TalonSRX right1;
    private TalonSRX right2;
    
    private TalonDrive driveTrain;
    private TankGyro gyro;
    private NavX talonGyro;
    
    
    private Joystick left;
    private Joystick right;
    
    private PowerDistributionPanel pdp = new PowerDistributionPanel(0);

    @Override
    public void robotInit() {
    	
    	// both left motors are inverted (left1 and left2)
    	// none of the sensors are inverted (setSensorPhase)
        left1 = new TalonSRX(57);
        left1.setInverted(true);
        left1.setSensorPhase(false);
        left2 = new TalonSRX(59);
        left2.setInverted(true);
        left2.setSensorPhase(false);
        right1 = new TalonSRX(58);
        right1.setSensorPhase(false);
        right2 = new TalonSRX(56);
        right2.setSensorPhase(false);
        
        left1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        left2.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        right1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);  
        right2.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        
    	gyro = new TankGyro(talonGyro,left1,left2,right1,right2); 

    	
        driveTrain = new TalonDrive(left1, left2, right1, right2);
        driveTrain.setMaxMotorRPM(5100);
        driveTrain.setWheelSize(6.0);
        driveTrain.setLowRatio(8.45);
        driveTrain.setHighRatio(8.45);
        driveTrain.setGear(Gear.LOW);
        driveTrain.setTicksPerRevolution(80);
        
        left = new Joystick(0);
        right = new Joystick(1);
    }

    @Override
    public void teleopPeriodic() {
        driveTrain.drive(left.getY(), right.getY());
        
        SmartDashboard.putNumber("Left", left.getY() * driveTrain.getMaxFloorSpeed());
        SmartDashboard.putNumber("Right", right.getY() * driveTrain.getMaxFloorSpeed());
        
        
        SmartDashboard.putNumber("left1", pdp.getCurrent(1));
        SmartDashboard.putNumber("left2", pdp.getCurrent(2));
        SmartDashboard.putNumber("right1", pdp.getCurrent(12));
        SmartDashboard.putNumber("right2", pdp.getCurrent(13));
        
        SmartDashboard.putNumber("left1 velocity", getFloorSpeed(left1.getSelectedSensorVelocity(0)));
        SmartDashboard.putNumber("left2 velocity", getFloorSpeed(left2.getSelectedSensorVelocity(0)));
        SmartDashboard.putNumber("right1 velocity", getFloorSpeed(right1.getSelectedSensorVelocity(0)));
        SmartDashboard.putNumber("right2 velocity", getFloorSpeed(right2.getSelectedSensorVelocity(0)));
    }
    
    @Override
	public void autonomousInit() {
    	gyro.turnToAngle(90.0);
    	gyro = new TankGyro(talonGyro,left1,left2,right1,right2); 
	}

	@Override
	public void autonomousPeriodic() {
		gyro.updateGyro();  
		SmartDashboard.putNumber("gyro angle", talonGyro.getNormalizedAngle());
	}

    
    private double getFloorSpeed(double sensorRdg) {
    	double motorRPM = sensorRdg / TalonDrive.SCALE_FACTOR / driveTrain.getTicksPerRevolution();
    	double wheelRPM = motorRPM / driveTrain.getGear().getRatio();
    	double wheelFtPerSec = wheelRPM * driveTrain.getWheelSize() / 60.0 / 12.0;
    	return wheelFtPerSec;
    }
    
}
