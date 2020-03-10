/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.custom.Ramper;
//import java.lang.Math;
import frc.robot.subsystems.DriveSubsystem;


public class DriveWithJoystick extends CommandBase {
  /**
   * Creates a new DriveWithJoystick.
   */
  //private final DriveSubsystem driveTrain;
  private final DriveSubsystem m_driveTrain;
  private final OI m_oi;
  private final Timer timer = new Timer();
  
  public DriveWithJoystick(DriveSubsystem driveTrain, OI oi) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    m_oi = oi;
    addRequirements(m_driveTrain);
    timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
    public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Declare variables for deadzone corrections
    double start = timer.get();

    final double turn = m_oi.getRotation();
    final double right = m_oi.getXValue();
    final double forward = m_oi.getYValue();
    final double sensitivity = m_oi.getSensitivity();
    m_driveTrain.driveWithJoystick(turn, right, forward, sensitivity);

    //System.out.println("DriveWithJoystic.java:execute():" + Double.toString(timer.get() - start));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
