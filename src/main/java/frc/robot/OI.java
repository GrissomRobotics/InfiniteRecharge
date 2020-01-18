/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.PositionControl;
import frc.robot.commands.DisableSpinner;

/**
 * Add your docs here.
 */
public class OI {

    public Joystick driveStick;
    public Joystick otherStick;
    
    
    public OI(){

        driveStick = new Joystick(0);
        otherStick = new Joystick(1);
        JoystickButton positionControlButton = new JoystickButton(otherStick, 4);
        JoystickButton positionControlStop = new JoystickButton(otherStick, 5);

        positionControlButton.whenPressed(new PositionControl());
        positionControlButton.whenPressed(new DisableSpinner());
    }

    public double getXValue() {
    	return driveStick.getX();// + (otherStick.getRawAxis(4)/otherSensitivity);
    }
    
    public double getYValue() {
    	return driveStick.getY();// + (otherStick.getRawAxis(5)/otherSensitivity);
    }
    
    
    public double getRotationLeft() {
    	return driveStick.getRawAxis(2);// + (otherStick.getRawAxis(2)/otherSensitivity);
    }
    
    public double getRotationRight() {
    	return driveStick.getRawAxis(3);// + (otherStick.getRawAxis(3)/otherSensitivity);
    }
}
