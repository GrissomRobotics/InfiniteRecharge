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
import frc.robot.commands.RotationControl;
import frc.robot.commands.Climb;
import frc.robot.commands.DisableSpinner;
import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class OI {

    public Joystick driveStick  = new Joystick(0);
    public Joystick otherStick  = new Joystick(1);
    public int camera_selection = 0;

    private final Timer timer = new Timer();

    public OI() {
    }

    public double getXValue() {
        return driveStick.getRawAxis(0);// + (otherStick.getRawAxis(4)/otherSensitivity);
    }

    public double getYValue() {
        return driveStick.getRawAxis(1);// + (otherStick.getRawAxis(5)/otherSensitivity);
    }

    public double getRotation() {
        return -driveStick.getRawAxis(2);// + (otherStick.getRawAxis(2)/otherSensitivity);
    }

    public double getSensitivity() {
        return (driveStick.getRawAxis(3)+1)/2;
    }

    // TODO: Map xbox axes

    public double getSpinnerRotation() {
        //System.out.println(otherStick.getRawAxis(3) - otherStick.getRawAxis(2));
        return otherStick.getRawAxis(3) - otherStick.getRawAxis(2);
    }

    public double getManualBeltRotation() {
        return otherStick.getRawAxis(5);
    }

    public double getManualArmRotation() {
        return otherStick.getRawAxis(1);
    }

}
