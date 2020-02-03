/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.IntakeSystem;

public class ManualIntakeArm extends CommandBase {
  /**
   * Creates a new ManualIntakeArm.
   */
  private final IntakeSystem m_intakeSystem;
  private final OI m_oi;

  public ManualIntakeArm(IntakeSystem intakeSystem, OI oi) {
    m_intakeSystem = intakeSystem;
    m_oi = oi;
    addRequirements(m_intakeSystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_intakeSystem.moveArmMotorManual(m_oi.getManualArmRotation());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSystem.stopArmMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
