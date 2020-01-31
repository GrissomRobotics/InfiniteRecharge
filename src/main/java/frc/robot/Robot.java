/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Spinner;


/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends TimedRobot {

private RobotMap robotMap;
  
  // Drive Train
  /*
   * private static final int kFrontLeftChannel = 2; private static final int
   * kRearLeftChannel = 3; private static final int kFrontRightChannel = 1;
   * private static final int kRearRightChannel = 0;
   * 
   * private MecanumDrive m_robotDrive; private Joystick m_stick;
   */

  /*
   * private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
   * private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
   * private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
   * private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524,
   * 0.113);
   */

  public Robot() {

    robotMap = new RobotMap();

    // CommandScheduler.getInstance().registerSubsystem(Robot.driveTrain);
  }

  @Override
  public void robotInit() {


    // CommandScheduler.getInstance().setDefaultCommand(driveTrain, new
    // DriveWithJoystick());

    /*
     * driveTrain = new DriveSubsystem(); spinner = new Spinner(); climber = new
     * Climber();
     * 
     * oi = new OI();
     */

    /*
     * 
     * Jaguar frontLeft = new Jaguar(kFrontLeftChannel); Jaguar rearLeft = new
     * Jaguar(kRearLeftChannel); Jaguar frontRight = new Jaguar(kFrontRightChannel);
     * Jaguar rearRight = new Jaguar(kRearRightChannel);
     * 
     * // Invert the left side motors. // You may need to change or remove this to
     * match your robot. frontLeft.setInverted(true); rearLeft.setInverted(true);
     * 
     * m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
     * 
     * m_stick = new Joystick(kJoystickChannel);
     */
    //CameraServer.getInstance().startAutomaticCapture();
  }

  public void robotPeriodic() {
    // SmartDashboard.putNumber("Confidence", match.confidence);
    // SmartDashboard.putString("Detected Color", colorString);
    // SmartDashboard.putBoolean("Color Match", colorMatched);
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {

  }

  public void autonomousInit() {

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
