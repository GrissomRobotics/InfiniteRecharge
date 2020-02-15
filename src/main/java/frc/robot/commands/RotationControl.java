/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.custom.WheelTracker;
import frc.robot.subsystems.Spinner;
import edu.wpi.first.wpilibj.Timer;

public class RotationControl extends CommandBase {
  /**
   * Creates a new RotationControl.
   */
  private final Spinner m_Spinner;
  private WheelTracker wheelTracker;
  private Timer timer;

  public RotationControl(Spinner spinner) {
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    m_Spinner = spinner;
    addRequirements(m_Spinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    // Color detectedColor = RobotMap.m_colorSensor.getColor();
    // Called every time the scheduler runs while the command is scheduled.
    wheelTracker= new WheelTracker(m_Spinner.getColor(), 4);

  }

  @Override
  public void execute() {
    final double start = timer.get();
    m_Spinner.spinPanelClockwise();
    wheelTracker.setNewColor(m_Spinner.getColor());
    System.out.println("RotationControl.java:execute():" + Double.toString(timer.get() - start));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Spinner.stopSpinner();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(wheelTracker.getNumRotations() >= 3.0)
      return true;
    return false;
    
  }
}
