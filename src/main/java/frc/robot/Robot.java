/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Spinner;
import frc.robot.commandGroups.*;
import edu.wpi.first.wpilibj.Timer;

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends TimedRobot {

  private RobotMap robotMap;

  // autonomous chooser
  private Command autonomousCommand;
  private SendableChooser<Command> chooser = new SendableChooser<Command>();
  private Timer timer = new Timer();

  public Robot() {
    timer.start();
    double start = timer.get();
  
    robotMap = new RobotMap();
  
    System.out.println("Robot.java:Robot():" + Double.toString(timer.get() - start));
  }

  @Override
  public void robotInit() {
    double start = timer.get();

    // Add commands to Autonomous Sendable Chooser

    chooser.setDefaultOption("Autonomous Default",
        new AutonomousDefault(robotMap.driveTrain, robotMap.spinner, robotMap.belt, robotMap.outputSystem));
    chooser.addOption("Autonomous From Side",
        new AutonomousFromSide(robotMap.driveTrain, robotMap.spinner, robotMap.belt, robotMap.outputSystem));
    chooser.addOption("Autonomous Get Off Line", new AutonomousOffLine(robotMap.driveTrain, robotMap.spinner));

    SmartDashboard.putData("Auto mode", chooser);

    System.out.println("Robot.java:robotInit():" + Double.toString(timer.get() - start));
  }

  public void robotPeriodic() {
    double start = timer.get();

    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Camera Selection:", robotMap.oi.camera_selection);

    //System.out.println("Robot.java:robotPeriodic():" + Double.toString(timer.get() - start));
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

  }

  public void autonomousInit() {

    robotMap.spinner.resetGyro();

    autonomousCommand = chooser.getSelected();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
    
  }

  public void autonomousPeriodic() {

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    // This must be here for sensor data to be updated to the dashboard while
    // disabled, do not remove it
    CommandScheduler.getInstance().run();
  }
}
