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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Spinner;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

public class DriveByUltrasonic extends CommandBase {
  /**
   * Creates a new DriveWithUltrasonic.
   */
  private final DriveSubsystem m_driveTrain;
  private final Spinner m_spinner;

  private static final double kP_Ultra = 1.0;
  private static final double kI_Ultra = 0.0;
  private static final double kD_Ultra = 0.0;
  private final PIDController pid_Ultra = new PIDController(kP_Ultra, kI_Ultra, kD_Ultra);
  
  private static final double kP_Gyro = 1.0;
  private static final double kI_Gyro = 0.0;
  private static final double kD_Gyro = 0.0;
  private final PIDController pid_Gyro = new PIDController(kP_Gyro, kI_Gyro, kD_Gyro);
  
  //private static PigeonIMU gyro;
  private static UltrasonicSensor ultra;
  private static double distanceSetpoint;
  private static double distanceTolerance;
  private static boolean commandIsFinished = false;

  private final Timer timer;

  //give input in inches
  public DriveByUltrasonic(DriveSubsystem driveTrain, Spinner spinner, double distance, double tolerance) {
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    m_spinner = spinner;
    pid_Gyro.setSetpoint(m_spinner.getGyroData());
    //converts input in inches to millimeters
    distanceSetpoint = distance * 25.400013716;
    distanceTolerance = tolerance;
    pid_Ultra.setSetpoint(distanceSetpoint);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double start = timer.get();
    double ultraDistance = m_driveTrain.getUltraReading();
    double gyroAngle = m_spinner.getGyroData();

    double driveRate = pid_Ultra.calculate(ultraDistance);
    double rotationRate = pid_Gyro.calculate(gyroAngle);
    driveRate = MathUtil.clamp(driveRate, -0.1, 0.1);
    rotationRate = MathUtil.clamp(rotationRate, -0.1, 0.1);

    if(Math.abs(ultraDistance-pid_Ultra.getSetpoint()) <= distanceTolerance){
      m_driveTrain.stop();
      commandIsFinished = true;
    }else{
      m_driveTrain.cartesianDrive(0.0, driveRate, rotationRate);
    }
    
    System.out.println("DriveByUltrasonic.java:execute():" + Double.toString(timer.get() - start));
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
