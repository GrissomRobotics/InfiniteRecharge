/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OutputSystem;

public class ToggleHatch extends CommandBase {
  /**
   * Creates a new ToggleHatch.
   */
  private final OutputSystem m_outputSystem;
  private boolean commandIsFinished = false;

  public ToggleHatch(OutputSystem outputSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_outputSystem = outputSystem;
    addRequirements(m_outputSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_outputSystem.hatchIsClosed()){
      m_outputSystem.openHatch();
    }else{
      m_outputSystem.closeHatch();
    }
    commandIsFinished = true;
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return commandIsFinished;
  }
}
