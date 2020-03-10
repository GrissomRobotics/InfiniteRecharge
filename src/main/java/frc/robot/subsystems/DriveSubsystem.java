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
import frc.robot.custom.Ramper;
import frc.robot.custom.UltrasonicObject;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;

public class DriveSubsystem extends SubsystemBase {

    // motors
    private final SpeedController leftFront;
    private final SpeedController rightFront;
    private final SpeedController leftRear;
    private final SpeedController rightRear;
    private final MecanumDrive mecanumDrive;

    // rampers
    private final double defaultRampStep = 0.05;
    private final Ramper rampForward = new Ramper(defaultRampStep);
    private final Ramper rampRight = new Ramper(defaultRampStep);
    
    // timer
    private final Timer timer = new Timer();

    //ultra
    UltrasonicObject m_ultra;


    public DriveSubsystem(UltrasonicObject ultra) {
        super();

        m_ultra = ultra;

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

        mecanumDrive.setSafetyEnabled(false);
        mecanumDrive.setExpiration(0.1);
        mecanumDrive.setMaxOutput(1.0);

    }

    @Override
    public void periodic() {

        //System.out.println("ultra: " + getUltraReading());

    }

    public void cartesianDrive(double xValue, double yValue, double rotationValue) {
        // Negate xValue to resolve strafing direction issue
        mecanumDrive.driveCartesian(-xValue, yValue, rotationValue);
    }

    public void driveWithJoystick(double turn, double right, double forward, double sensitivity) {

        double turnSet;
        double forwardSet;
        double rightSet;
        double rightThreshold = 0.1;
        double deadThreshold = 0.1;

        if (Math.abs(turn) > deadThreshold) {
            turn = Math.signum(turn)*turn*turn;
            turnSet = turn;
        } else {
            turnSet = 0;
        }

        if (Math.abs(forward) > deadThreshold) {
            forward = Math.signum(forward)*forward*forward;
            forwardSet = rampForward.ramp(forward);
        } else {
            forwardSet = 0;
        }

        if (Math.abs(right) > rightThreshold) {
            right= Math.signum(right)*right*right;
            rightSet = rampRight.ramp(right);
        } else {
            rightSet = 0;
        }

        cartesianDrive(rightSet*sensitivity, forwardSet*sensitivity, turnSet*0.6);
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

    public void setSafetyEnabled(boolean isOn){
        mecanumDrive.setSafetyEnabled(isOn);
    }

}
