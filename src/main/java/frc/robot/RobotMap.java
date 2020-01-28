/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.*;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Climb;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.custom.UltrasonicSensor;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Spinner;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import com.revrobotics.ColorSensorV3;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort.StopBits;

/**
 * Add your docs here.
 */
public class RobotMap {

    private final DriveSubsystem driveTrain = new DriveSubsystem();
    private final Spinner spinner = new Spinner();
    private final Climber climber = new Climber();
    private final OI oi = new OI();
    private final Joystick driveStick = new Joystick(0);
    private final Joystick otherStick = new Joystick(1);

    // spinner and color detection stuff
    public static I2C.Port i2cPort;
    public static ColorSensorV3 m_colorSensor;

    // sensors
    public static PigeonIMU gyro;
    public static UltrasonicSensor ultra;
    public static SerialPort ultraSerial;

    // constants
    public static final double SPINNER_WHEEL_SPEED = 0.5;
    public static final double WINCH_SPEED = 0.5;

    public RobotMap() {

        // spinner and color sensor
        //i2cPort = I2C.Port.kOnboard;
        //m_colorSensor = new ColorSensorV3(i2cPort);

        // sensor
        //ultraSerial = new SerialPort(9600, Port.kOnboard, 8, Parity.kNone, StopBits.kOne);
        //ultraSerial.reset();

        //gyro = new PigeonIMU(0);
        //ultra = new UltrasonicSensor(ultraSerial);

        driveTrain.setDefaultCommand(new DriveWithJoystick(driveTrain, oi));

    }

    private void configureButtonBindings() {
        // buttons
        final JoystickButton climbButton = new JoystickButton(driveStick, 4);

        final JoystickButton positionControlButton = new JoystickButton(otherStick, 4);
        final JoystickButton rotationControlButton = new JoystickButton(otherStick, 1);
        final JoystickButton cancelSpinnerButton = new JoystickButton(otherStick, 3);

        // buttons to commands
        climbButton.whileHeld(new Climb(climber));

        positionControlButton.whenPressed(new PositionControl(spinner));
        rotationControlButton.whenPressed(new RotationControl(spinner));
        cancelSpinnerButton.whenPressed(new DisableSpinner(spinner));

    }

}
