/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.custom.UltrasonicSensor;

import com.ctre.phoenix.sensors.PigeonIMU;
import frc.robot.custom.UltrasonicSensor;

public class DriveSubsystem extends SubsystemBase {

    //motors

    private final SpeedController leftFront = RobotMap.driveTrainLeftFront;
    private final SpeedController rightFront = RobotMap.driveTrainRightFront;
    private final SpeedController leftRear = RobotMap.driveTrainLeftRear;
    private final SpeedController rightRear = RobotMap.driveTrainRightRear;
    protected final MecanumDrive mecanumDrive = RobotMap.driveTrainMecanumDrive;
    public final double defaultRampStep = 0.01;

    //gyro
    private PigeonIMU gyro = RobotMap.gyro;
    private UltrasonicSensor ultra = RobotMap.ultra;

    public void initDefaultCommand() {
        setDefaultCommand(new DriveWithJoystick());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro", gyro.getFusedHeading());
        SmartDashboard.putNumber("Ultra", ultra.readLastRange());

    }

    public void cartesianDrive(double xValue, double yValue, double rotationValue) {
        // Negate xValue to resolve strafing direction issue
        mecanumDrive.driveCartesian(-xValue, yValue, rotationValue);
    }

    public void moveForward() {
        leftFront.set(0.5);
        rightFront.set(0.5);
        leftRear.set(0.5);
        rightRear.set(0.5);

    }

    public void stop() {
        leftFront.stopMotor();
        rightFront.stopMotor();
        leftRear.stopMotor();
        rightRear.stopMotor();

    }

    public void resetGyro() {
		gyro.setFusedHeading(0);
	}

}
