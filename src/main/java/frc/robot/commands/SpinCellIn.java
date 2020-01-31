/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSystems;;

public class SpinCellIn extends CommandBase {
  /**
   * Creates a new SpinCellIn.
   */
  private final IntakeSystems m_IntakeSystems;

  public SpinCellIn(IntakeSystems intakeSystems) {
    m_IntakeSystems = intakeSystems;
     addRequirements(m_IntakeSystems);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  
  @Override
  public void initialize() {
    m_IntakeSystems.spinWheelCClockwise();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSystems.spinWheelClockwise();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSystems.spinWheelOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
