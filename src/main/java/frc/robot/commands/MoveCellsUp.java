/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Belt;
public class MoveCellsUp extends CommandBase {
  /**
   * Creates a new MoveCellsUp.
   */
  private final Belt m_belt;

  public MoveCellsUp(Belt belt) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_belt = belt;
    addRequirements(belt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_belt.turnBeltCClockwise();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_belt.turnBeltClockwise();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  m_belt.turnBeltOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
 
  
}
