/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.IntakeSystem;

public class IntakeAll extends CommandBase {
  /**
   * Creates a new IntakeAll.
   */
  private final IntakeSystem m_intakeSystem;
  private final Belt m_belt;

  public IntakeAll(IntakeSystem intakeSystem, Belt belt) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_belt = belt;
    m_intakeSystem = intakeSystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_belt.turnBelt(1);
    m_intakeSystem.setSpinWheel(1);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSystem.spinWheelOff();
    m_belt.turnBeltOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
