/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
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
    chooser.addOption("Autonomous Get Off Line", 
        new AutonomousOffLine(robotMap.driveTrain));
    chooser.addOption("Timed Autonomous Default",
        new TimedAutonomousDefault(robotMap.driveTrain, robotMap.belt, robotMap.outputSystem));
    chooser.addOption("Timed Autonomoud From Side",
        new TimedAutonomousFromSide(robotMap.driveTrain, robotMap.spinner, robotMap.belt, robotMap.outputSystem));

    SmartDashboard.putData("Auto mode", chooser);

    Thread t = new Thread(() -> {
    		
      boolean allowCam1 = false;
      
      UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(0);
          camera1.setResolution(160, 120);
          camera1.setFPS(30);
          UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(1);
          camera2.setResolution(160, 120);
          camera2.setFPS(30);
          
          CvSink cvSink1 = CameraServer.getInstance().getVideo(camera1);
          CvSink cvSink2 = CameraServer.getInstance().getVideo(camera2);
          CvSource outputStream = CameraServer.getInstance().putVideo("Switcher", 320, 240);
          
          Mat image = new Mat();
          
          while(!Thread.interrupted()) {
            
            if(robotMap.cameraSelection()) {
              allowCam1 = !allowCam1;
            }
            
              if(allowCam1){
                cvSink2.setEnabled(false);
                cvSink1.setEnabled(true);
                cvSink1.grabFrame(image);
              } else{
                cvSink1.setEnabled(false);
                cvSink2.setEnabled(true);
                cvSink2.grabFrame(image);     
              }
              
              outputStream.putFrame(image);
          }
          
      });
      t.start();

    

    // UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    // camera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 120);
    // new Thread(() -> {
    //   CvSink cvSink = CameraServer.getInstance().getVideo();
    //   CvSource outputStream = CameraServer.getInstance().putVideo("camera stream", 320, 240);
    //   Mat source = new Mat();
    //   while (!Thread.interrupted()) {
    //     cvSink.grabFrame(source);
    //     outputStream.putFrame(source);
    //   }
    // }).start();

    System.out.println("Robot.java:robotInit():" + Double.toString(timer.get() - start));
  }

  public void robotPeriodic() {
    double start = timer.get();

    CommandScheduler.getInstance().run();

    // SmartDashboard.putNumber("Camera Selection:", robotMap.oi.camera_selection);

    // System.out.println("Robot.java:robotPeriodic():" +
    // Double.toString(timer.get() - start));
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    robotMap.driveTrain.setSafetyEnabled(true);
  }

  @Override
  public void teleopPeriodic() {

  }

  public void autonomousInit() {

    robotMap.spinner.resetGyro();

    robotMap.driveTrain.setSafetyEnabled(false);

    autonomousCommand = chooser.getSelected();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }

  }

  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  public void testInit() {

    // robotMap.spinner.resetGyro();

    // autonomousCommand = chooser.getSelected();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }

  }

  public void testPeriodic() {
    CommandScheduler.getInstance().run();

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
