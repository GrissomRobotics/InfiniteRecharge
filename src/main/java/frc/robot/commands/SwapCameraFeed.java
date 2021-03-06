/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.RobotMap;

public class SwapCameraFeed extends CommandBase {
  /**
   * Creates a new SwapCameraFeed.
   */
  OI m_oi;

  public SwapCameraFeed(OI oi) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_oi = oi;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_oi.camera_selection = (m_oi.camera_selection == 0) ? 0 : 1;    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
