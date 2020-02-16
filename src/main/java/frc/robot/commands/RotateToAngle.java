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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Spinner;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.controller.PIDController;

import edu.wpi.first.wpiutil.math.MathUtil;

import edu.wpi.first.wpilibj.Timer;

public class RotateToAngle extends CommandBase {
  /**
   * Creates a new RotateToAngle.
   */
  private final DriveSubsystem m_driveTrain;
  private final Spinner m_spinner;
 
  private static final double kP = 1.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private final PIDController pid = new PIDController(kP, kI, kD);
  private static boolean commandIsFinished = false;
  private static double angleTolerance;
  private final Timer timer = new Timer();

  public RotateToAngle(DriveSubsystem driveTrain, Spinner spinner, double angle, double tolerance) {
    // Use addRequirements() here to declare subsystem dependencies.  
    m_driveTrain = driveTrain;
    m_spinner = spinner;
    angleTolerance = tolerance;
    pid.setSetpoint(angle);
    timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double start = timer.get();
    double gyroAngle = m_spinner.getGyroData();
    double rotationRate = pid.calculate(gyroAngle);
    rotationRate = MathUtil.clamp(rotationRate, -0.1, 0.1);

    if(Math.abs(gyroAngle-pid.getSetpoint()) <= angleTolerance){
      m_driveTrain.stop();
      commandIsFinished = true;
    } else{
      m_driveTrain.cartesianDrive(0.0, 0.0, rotationRate);
    }
    //System.out.println("RotateToAngle.java:execute():" + Double.toString(timer.get() - start));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return commandIsFinished;
  }
}
