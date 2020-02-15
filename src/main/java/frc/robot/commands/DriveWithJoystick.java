/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.custom.Ramper;
//import java.lang.Math;
import frc.robot.subsystems.DriveSubsystem;


public class DriveWithJoystick extends CommandBase {
  /**
   * Creates a new DriveWithJoystick.
   */
  //private final DriveSubsystem driveTrain;
  private final DriveSubsystem m_driveTrain;
  private final OI m_oi;

  private Ramper rampForward;
  private Ramper rampRight;
  private Timer timer;
  
  public DriveWithJoystick(DriveSubsystem driveTrain, OI oi) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    m_oi = oi;
    timer = new Timer();
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
    public void initialize() {
      timer.start();
      rampForward = new Ramper(m_driveTrain.defaultRampStep); 
    	rampRight = new Ramper(m_driveTrain.defaultRampStep); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Declare variables for deadzone corrections
    double start = timer.get();
    double turn;
    double forward;
    double right;
    double turnSet;
    double forwardSet;
    double rightSet;
    double rightThreshold = 0.1;
    double deadThreshold = 0.1;
    

    // Correct deadzones
    // Logic is: if the r

    // reading is greater than the threshold, make the setter equal to it,
    // otherwise, make the setter equal to 0
    turn = m_oi.getRotation();
    right = m_oi.getXValue();
    forward = m_oi.getYValue();

    if (Math.abs(turn) > deadThreshold) {
      turnSet = turn;
    } else {
      turnSet = 0;
    }

    if (Math.abs(forward) > deadThreshold) {
      forwardSet = rampForward.ramp(forward);
    } else {
      forwardSet = 0;
    }

    if (Math.abs(right) > rightThreshold) {
      rightSet = rampRight.ramp(right);
    } else {
      rightSet = 0;
    }

    m_driveTrain.cartesianDrive(rightSet, forwardSet, (turnSet * 0.6));

    double end = timer.get();
    double dtTime = end-start;
    System.out.println("Drive Timer:" + dtTime);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
