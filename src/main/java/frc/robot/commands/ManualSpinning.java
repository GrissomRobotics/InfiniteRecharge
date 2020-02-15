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
import frc.robot.Robot;
import frc.robot.subsystems.Spinner;

public class ManualSpinning extends CommandBase {
  /**
   * Creates a new ManualSpinning.
   */
  private final Spinner m_Spinner;
  private final OI m_oi;
  private Timer timer;

  public ManualSpinning(Spinner spinner, OI oi) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_oi = oi;
    m_Spinner = spinner;
    timer = new Timer(); 
    addRequirements(m_Spinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double start = timer.get();
    m_Spinner.spinManual(m_oi.getSpinnerRotation());
    double end = timer.get();
    double dtTime = end-start;
    System.out.println("Spinner Timer:" + dtTime);
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Spinner.stopSpinner();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
