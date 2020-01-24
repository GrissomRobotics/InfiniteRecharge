/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.revrobotics.ColorSensorV3;
import com.ctre.phoenix.sensors.PigeonIMU;

/**
 * Add your docs here.
 */
public class RobotMap {

    // spinner and color detection stuff
    public static I2C.Port i2cPort;
    public static ColorSensorV3 m_colorSensor;
    public static PWMVictorSPX spinnerWheel;

    // climber subsystem
    public static SpeedController climberWinch;

    // drivetrain subsystem
    public static SpeedController driveTrainLeftFront;
    public static SpeedController driveTrainRightFront;
    public static SpeedController driveTrainLeftRear;
    public static SpeedController driveTrainRightRear;
    public static MecanumDrive driveTrainMecanumDrive;

    // sensors
	public static PigeonIMU gyro;

    // constants
    public static final double SPINNER_WHEEL_SPEED = 0.5;
    public static final double WINCH_SPEED = 0.5;

    public static void init() {

        // spinner subsystem
        spinnerWheel = new PWMVictorSPX(4);

        //climber
        climberWinch = new PWMVictorSPX(5);

        // drive train
        driveTrainLeftFront = new Jaguar(0);
        driveTrainLeftFront.setInverted(true);

        driveTrainRightFront = new Jaguar(1);
        driveTrainRightFront.setInverted(false);

        driveTrainLeftRear = new Jaguar(2);
        driveTrainLeftRear.setInverted(true);

        driveTrainRightRear = new Jaguar(3);
        driveTrainRightRear.setInverted(false);

        driveTrainMecanumDrive = new MecanumDrive(driveTrainLeftFront, driveTrainLeftRear, driveTrainRightFront,
                driveTrainRightRear);

        driveTrainMecanumDrive.setSafetyEnabled(true);
        driveTrainMecanumDrive.setExpiration(0.1);
        driveTrainMecanumDrive.setMaxOutput(1.0);

        // spinner and color sensor
        i2cPort = I2C.Port.kOnboard;
        m_colorSensor = new ColorSensorV3(i2cPort);

        // sensor
        //sensors
		gyro = new PigeonIMU(0);
    }
}
