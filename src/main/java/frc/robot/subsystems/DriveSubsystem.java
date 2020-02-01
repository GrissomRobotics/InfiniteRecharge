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
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.custom.Ramper;
import frc.robot.custom.UltrasonicSensor;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;

import com.ctre.phoenix.sensors.PigeonIMU;
import frc.robot.custom.UltrasonicSensor;

public class DriveSubsystem extends SubsystemBase {

    // motors
    private final SpeedController leftFront;
    private final SpeedController rightFront;
    private final SpeedController leftRear;
    private final SpeedController rightRear;
    private final MecanumDrive mecanumDrive;
    public final double defaultRampStep = 0.01;

    // gyro
    private PigeonIMU gyro = RobotMap.gyro;
    private UltrasonicSensor ultra = RobotMap.ultra;

    // rampers
    private Ramper rampForward;
    private Ramper rampRight;

    public DriveSubsystem() {

        // sensor
        // ultraSerial = new SerialPort(9600, Port.kOnboard, 8, Parity.kNone,
        // StopBits.kOne);
        // ultraSerial.reset();

        // gyro = new PigeonIMU(0);
        // ultra = new UltrasonicSensor(ultraSerial);

        // drive train
        // sides weree going in different directions, so not inverting left side.
        // this might need to be undone later
        // now they have all been inverted so that it should go forwards instead of
        // backwards
        leftFront = new PWMVictorSPX(0);
        leftFront.setInverted(true);

        rightFront = new PWMVictorSPX(1);
        rightFront.setInverted(true);

        leftRear = new PWMVictorSPX(2);
        leftRear.setInverted(true);

        rightRear = new PWMVictorSPX(3);
        rightRear.setInverted(true);

        mecanumDrive = new MecanumDrive(leftFront, leftRear, rightFront, rightRear);

        mecanumDrive.setSafetyEnabled(true);
        mecanumDrive.setExpiration(0.1);
        mecanumDrive.setMaxOutput(1.0);
        rampForward = new Ramper(defaultRampStep);
        rampRight = new Ramper(defaultRampStep);

    }

    @Override
    public void periodic() {
        // TODO: Uncomment when ultra and gyro added for debugging purposes
        // SmartDashboard.putNumber("Gyro", gyro.getFusedHeading());
        // SmartDashboard.putNumber("Ultra", ultra.readLastRange());

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

        // Correct deadzones
        // Logic is: if the r

        // reading is greater than the threshold, make the setter equal to it,
        // otherwise, make the setter equal to 0
        // turn = Robot.oi.getRotationLeft() - Robot.oi.getRotationRight();
        // right = Robot.oi.getXValue();
        // forward = Robot.oi.getYValue();

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

        cartesianDrive(rightSet, forwardSet, (turnSet * 0.6));
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
