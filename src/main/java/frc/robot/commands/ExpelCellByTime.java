/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.OutputSystem;

public class ExpelCellByTime extends CommandBase {
  /**
   * Creates a new ExpelCellByTime.
   */
  private final Belt m_belt;
  private final OutputSystem m_outputSystem;
  private static Timer timer;
  private static double delayTime;

  public ExpelCellByTime(Belt belt, OutputSystem outputSystem, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_belt = belt;
    m_outputSystem = outputSystem;
    addRequirements(m_belt, m_outputSystem);
    delayTime = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_outputSystem.openDoor();;
    m_belt.turnBeltClockwise();
    Timer.delay(delayTime);
    m_belt.turnBeltOff();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_belt.turnBeltOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
