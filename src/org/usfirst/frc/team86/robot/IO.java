package org.usfirst.frc.team86.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class IO {
   public static Joystick coJoystick = new Joystick(2);
	

	
	public static Button rotateButton = new JoystickButton(coJoystick, 1);
	public static Button resetButton = new JoystickButton(coJoystick, 2);

}
