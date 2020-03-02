/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSystem;

public class ToggleArm extends CommandBase {
  /**
   * Creates a new ArmToggle.
   */
  IntakeSystem m_intakeSystem;

  public ToggleArm(IntakeSystem intakeSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intakeSystem = intakeSystem;
    addRequirements(m_intakeSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ThE ArM is Toggling!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    if(m_intakeSystem.getLowerLimitSwitch()){
      m_intakeSystem.moveArmUp();
    }else{
      m_intakeSystem.moveArmMotorDown();      
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_intakeSystem.getLowerLimitSwitch() || m_intakeSystem.getUpperLimitSwitch()){
      m_intakeSystem.stopArmMotor();
    }
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
