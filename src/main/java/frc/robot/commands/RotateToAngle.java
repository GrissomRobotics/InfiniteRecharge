/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.controller.PIDController;

public class RotateToAngle extends CommandBase {
  /**
   * Creates a new RotateToAngle.
   */
 
  private static final double kP = 1.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private final PIDController pid = new PIDController(kP, kI, kD);
  
  
  private static PigeonIMU gyro; 

  public RotateToAngle(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.  
    gyro = RobotMap.gyro;
    pid.setSetpoint(angle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double gyroAngle = gyro.getFusedHeading();
    double rotationRate = pid.calculate(gyroAngle);
    Robot.driveTrain.cartesianDrive(0.0, 0.0, rotationRate);    
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
