/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.custom.Ramper;
import frc.robot.custom.UltrasonicObject;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;

//import com.ctre.phoenix.sensors.PigeonIMU;

public class DriveSubsystem extends SubsystemBase {

    // motors
    private final SpeedController leftFront;
    private final SpeedController rightFront;
    private final SpeedController leftRear;
    private final SpeedController rightRear;
    private final MecanumDrive mecanumDrive;

    // rampers
    private final double defaultRampStep = 0.01;
    private final Ramper rampForward = new Ramper(defaultRampStep);
    private final Ramper rampRight = new Ramper(defaultRampStep);
    
    // timer
    private final Timer timer = new Timer();

    //ultra
    UltrasonicObject m_ultra;


    public DriveSubsystem(UltrasonicObject ultra) {
        super();
        timer.start();
        double start = timer.get();
        m_ultra = ultra;

        // gyro = new PigeonIMU(1);

        // drive train
        // sides were going in different directions, so not inverting left side.

        leftFront = new PWMVictorSPX(5);
        leftFront.setInverted(true);

        rightFront = new PWMVictorSPX(0);
        rightFront.setInverted(true);

        leftRear = new PWMVictorSPX(4);
        leftRear.setInverted(true);

        rightRear = new PWMVictorSPX(1);
        rightRear.setInverted(true);

        mecanumDrive = new MecanumDrive(leftFront, leftRear, rightFront, rightRear);

        mecanumDrive.setSafetyEnabled(true);
        mecanumDrive.setExpiration(0.1);
        mecanumDrive.setMaxOutput(1.0);

        System.out.println("DriverSubsystem.java:DriveSubsystem():" + Double.toString(timer.get() - start));
    }

    @Override
    public void periodic() {

        System.out.println("ultra: " + getUltraReading());

    }

    public void cartesianDrive(double xValue, double yValue, double rotationValue) {
        // Negate xValue to resolve strafing direction issue
        mecanumDrive.driveCartesian(-xValue, yValue, rotationValue);
    }

    public void driveWithJoystick(double turn, double right, double forward) {

        double turnSet;
        double forwardSet;
        double rightSet;
        double rightThreshold = 0.1;
        double deadThreshold = 0.1;

        if (Math.abs(turn) > deadThreshold) {
            turnSet = turn;
        } else {
            turnSet = 0;
        }

        if (Math.abs(forward) > deadThreshold) {
            forwardSet = rampForward.ramp(forward);
        } else {
            forwardSet = 0;
        }

        if (Math.abs(right) > rightThreshold) {
            rightSet = rampRight.ramp(right);
        } else {
            rightSet = 0;
        }

        cartesianDrive(rightSet, forwardSet, turnSet * 0.6);
    }

    public void stop() {
        leftFront.stopMotor();
        rightFront.stopMotor();
        leftRear.stopMotor();
        rightRear.stopMotor();
    }

    // reads in inches
    public double getUltraReading() {
        return m_ultra.getLastRange();
    }

}
