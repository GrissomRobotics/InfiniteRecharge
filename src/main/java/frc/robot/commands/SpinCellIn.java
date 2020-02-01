/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSystem;

public class SpinCellIn extends CommandBase {
  /**
   * Creates a new SpinCellIn.
   */
  private final IntakeSystem m_IntakeSystem;

  public SpinCellIn(IntakeSystem intakeSystem) {
    m_IntakeSystem = intakeSystem;
    addRequirements(m_IntakeSystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
    m_IntakeSystem.spinWheelCClockwise();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSystem.spinWheelClockwise();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSystem.spinWheelOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
