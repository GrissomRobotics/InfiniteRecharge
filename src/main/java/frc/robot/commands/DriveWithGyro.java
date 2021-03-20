/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Spinner;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.controller.PIDController;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.custom.UltrasonicSensor;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

public class DriveWithGyro extends CommandBase {
  /**
   * Creates a new DriveWithGyro.
   */

  private final DriveSubsystem m_driveTrain;
  private final Spinner m_spinner;

  
  private static final double kP_Gyro = 0.5;
  private static final double kI_Gyro = 0.0;
  private static final double kD_Gyro = 0.2;
  private final PIDController pid_Gyro = new PIDController(kP_Gyro, kI_Gyro, kD_Gyro);
  
  //private static PigeonIMU gyro;
  private static UltrasonicSensor ultra;
  private static double distanceSetpoint;
  private static double distanceTolerance;
  private static boolean commandIsFinished = false;

  private final Timer timer = new Timer();

  public DriveWithGyro(DriveSubsystem driveTrain, Spinner spinner) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    m_spinner = spinner;
    pid_Gyro.setSetpoint(m_spinner.getGyroData());
    timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("DRIVE WITH GYRO RUNNING");
    final double start = timer.get();
    double gyroAngle = m_spinner.getGyroData();
    double rotationRate = pid_Gyro.calculate(gyroAngle);
  

    if(timer.get() >= 5){
      m_driveTrain.stop();
      commandIsFinished = true;
    } else {
      m_driveTrain.cartesianDrive(0.5, 0, rotationRate);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return commandIsFinished;
  }
}
