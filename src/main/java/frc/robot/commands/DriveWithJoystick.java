/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.custom.Ramper;

public class DriveWithJoystick extends CommandBase {
  /**
   * Creates a new DriveWithJoystick.
   */

  private Ramper rampForward;
  private Ramper rampRight;
  
  public DriveWithJoystick() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
    public void initialize() {
      rampForward = new Ramper(Robot.driveTrain.defaultRampStep); 
    	rampRight = new Ramper(Robot.driveTrain.defaultRampStep); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Declare variables for deadzone corrections
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
    turn = Robot.oi.getRotationLeft() - Robot.oi.getRotationRight();
    right = Robot.oi.getXValue();
    forward = Robot.oi.getYValue();

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

    Robot.driveTrain.cartesianDrive(rightSet, forwardSet, (turnSet * 0.6));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
