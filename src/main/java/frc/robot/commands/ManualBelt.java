/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Belt;

public class ManualBelt extends CommandBase {
  /**
   * Creates a new ManualBelt. 
   */
  
  private final Belt m_belt;
  private final OI m_oi;
  private final Timer timer = new Timer();

  public ManualBelt(Belt belt, OI oi) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_belt = belt;
    m_oi = oi;
    addRequirements(m_belt);
    timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double start = timer.get();
    m_belt.turnBelt(m_oi.getManualBeltRotation());
    //System.out.println("ManualBelt.java:execute():" + Double.toString(timer.get() - start));
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
