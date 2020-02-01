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

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends TimedRobot {

  private RobotMap robotMap;

  // autonomous chooser
  private Command autonomousCommand;
  private SendableChooser<Command> chooser = new SendableChooser<Command>();

  public Robot() {

    robotMap = new RobotMap();

  }

  @Override
  public void robotInit() {

    // Add commands to Autonomous Sendable Chooser

    chooser.setDefaultOption("Autonomous Default", new AutonomousFromSide(robotMap.driveTrain));
    chooser.addOption("Autonomous Get Off Line", new AutonomousOffLine());

    SmartDashboard.putData("Auto mode", chooser);
  }

  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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

    autonomousCommand = chooser.getSelected();

    /*
    switch(chooser.getSelected()){
      case 1:
        System.out.println("PlaceHolder");//autonomousCommand = new AutonomousDefault(robotMap.driveTrain);
      case 2:
        System.out.println("PlaceHolder");//autonomousCommand = new AutonomousFromSide(robotMap.driveTrain);
      case 3:
        System.out.println("PlaceHolder");//autonomousCommand = new AutonomousOffLine(robotMap.driveTrain);
    }
    */


    // schedule the autonomous command (example)
    if (autonomousCommand != null){
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
