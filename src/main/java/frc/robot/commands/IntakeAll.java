/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.IntakeSystem;

public class IntakeAll extends CommandBase {
  /**
   * Creates a new IntakeAll.
   */
  private final IntakeSystem m_intakeSystem;
  private final Belt m_belt;
  private final OI m_oi;

  public IntakeAll(IntakeSystem intakeSystem, Belt belt, OI oi) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_belt = belt;
    m_intakeSystem = intakeSystem;
    m_oi = oi;
    addRequirements(m_intakeSystem, m_belt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_belt.turnBelt(m_oi.getManualBeltRotation());
    m_intakeSystem.setSpinWheel(m_oi.getManualBeltRotation());
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
