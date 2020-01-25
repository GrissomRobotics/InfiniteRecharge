/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.custom.UltrasonicSensor;
import edu.wpi.first.wpiutil.math.MathUtil;

public class DriveWithUltrasonic extends CommandBase {
  /**
   * Creates a new DriveWithUltrasonic.
   */

  private static final double kP_Ultra = 1.0;
  private static final double kI_Ultra = 0.0;
  private static final double kD_Ultra = 0.0;
  private final PIDController pid_Ultra = new PIDController(kP_Ultra, kI_Ultra, kD_Ultra);
  
  private static final double kP_Gyro = 1.0;
  private static final double kI_Gyro = 0.0;
  private static final double kD_Gyro = 0.0;
  private final PIDController pid_Gyro = new PIDController(kP_Gyro, kI_Gyro, kD_Gyro);
  
  private static PigeonIMU gyro;
  private static UltrasonicSensor ultra;
  private static double distanceSetpoint;

  public DriveWithUltrasonic(double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    gyro = RobotMap.gyro;
    pid_Gyro.setSetpoint(gyro.getFusedHeading());
    //converts input in inches to millimeters
    distanceSetpoint = distance * 25.400013716;
    ultra = RobotMap.ultra;
    pid_Ultra.setSetpoint(distanceSetpoint);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ultraDistance = ultra.readLastRange();
    double gyroAngle = gyro.getFusedHeading();

    double driveRate = pid_Ultra.calculate(ultraDistance);
    double rotationRate = pid_Gyro.calculate(gyroAngle);
    driveRate = MathUtil.clamp(driveRate, -0.1, 0.1);
    Robot.driveTrain.cartesianDrive(0.0, driveRate, rotationRate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}