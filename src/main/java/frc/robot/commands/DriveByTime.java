/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriveSubsystem;

public class DriveByTime extends CommandBase {
  /**
   * Creates a new AutonomousCommand.
   */
  private final DriveSubsystem m_driveTrain;
  private double driveTime;

  // give time in seconds
  public DriveByTime(DriveSubsystem driveTrain, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    addRequirements(m_driveTrain);
    driveTime = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.cartesianDrive(0.0, 0.5, 0.0);
    Timer.delay(driveTime);
    m_driveTrain.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
