/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Spinner;

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends TimedRobot {

  //subsystems
  public static RobotMap RobotMap;
	public static DriveSubsystem driveTrain;
	public static Spinner spinner;
	public static Climber climber;

	//OI	
	public static OI oi;


  //Drive Train
  /*
  private static final int kFrontLeftChannel = 2;
  private static final int kRearLeftChannel = 3;
  private static final int kFrontRightChannel = 1;
  private static final int kRearRightChannel = 0;

  private MecanumDrive m_robotDrive;
  private Joystick m_stick;
  */

  /*
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  */
  public Robot(){

    RobotMap = new RobotMap();

    driveTrain = new DriveSubsystem();
    spinner = new Spinner();
    climber = new Climber();

    oi = new OI();
    CommandScheduler.getInstance().registerSubsystem(driveTrain);
  }

  @Override
  public void robotInit() {

    /*
    driveTrain = new DriveSubsystem();
    spinner = new Spinner();
    climber = new Climber();

    oi = new OI();
    */


    /*

    Jaguar frontLeft = new Jaguar(kFrontLeftChannel);
    Jaguar rearLeft = new Jaguar(kRearLeftChannel);
    Jaguar frontRight = new Jaguar(kFrontRightChannel);
    Jaguar rearRight = new Jaguar(kRearRightChannel);

    // Invert the left side motors.
    // You may need to change or remove this to match your robot.
    frontLeft.setInverted(true);
    rearLeft.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    m_stick = new Joystick(kJoystickChannel);
    */

  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.

    
    //m_robotDrive.driveCartesian(m_stick.getY(), -m_stick.getZ(), m_stick.getX(), 0.0);
  }

  public void robotPeriodic() {
    //SmartDashboard.putNumber("Confidence", match.confidence);
    //SmartDashboard.putString("Detected Color", colorString);
    //SmartDashboard.putBoolean("Color Match", colorMatched);
    CommandScheduler.getInstance().run();
  }
}
