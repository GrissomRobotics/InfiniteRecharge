/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.*;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.custom.UltrasonicObject;
import frc.robot.custom.UltrasonicSensor;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.OutputSystem;
import frc.robot.subsystems.Spinner;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import com.revrobotics.ColorSensorV3;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort.StopBits;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.Timer;

public class RobotMap {

    // ultra
    public Thread m_ultraThread;
    private SerialPort ultraSerial = new SerialPort(9600, Port.kMXP, 8, Parity.kNone, StopBits.kOne);
    private UltrasonicSensor ultrasonicSensor = new UltrasonicSensor(ultraSerial);
    private UltrasonicObject ultra = new UltrasonicObject(ultrasonicSensor);

    // subsystems
    public final DriveSubsystem driveTrain = new DriveSubsystem(ultra);
    public final Spinner spinner = new Spinner();
    private final Climber climber = new Climber();
    public final Belt belt = new Belt();
    private final IntakeSystem intakeSystem = new IntakeSystem();
    public final OutputSystem outputSystem = new OutputSystem();

    // input stuff
    public final OI oi = new OI();
    private final Joystick driveStick = new Joystick(0);
    private final Joystick otherStick = new Joystick(1);

    // camera
    public Thread m_visionThread;
    public int camera_selection = 0;

    public RobotMap() {

        configureButtonBindings();
        belt.setDefaultCommand(new ManualBelt(belt, oi));
        intakeSystem.setDefaultCommand(new ManualIntakeArm(intakeSystem, oi));
        spinner.setDefaultCommand(new ManualSpinning(spinner, oi));
        driveTrain.setDefaultCommand(new DriveWithJoystick(driveTrain, oi));

        // All the vision stuff

        // m_visionThread = new Thread(() -> {
        //     // Get the UsbCamera from CameraServer
        //     UsbCamera camera0 = CameraServer.getInstance().startAutomaticCapture(0);
        //     UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(1);
        //     System.out.println("camera thread did a thing!");
        //     // Set the resolution
        //     camera0.setResolution(160, 120);
        //     camera1.setResolution(160, 120);

        //     // Get a CvSink. This will capture Mats from the camera
        //     CvSink cvSink0 = CameraServer.getInstance().getVideo(camera0);
        //     CvSink cvSink1 = CameraServer.getInstance().getVideo(camera1);
        //     // Setup a CvSource. This will send images back to the Dashboard
        //     CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 640, 480);

        //     // Mats are very memory expensive. Lets reuse this Mat.
        //     Mat mat = new Mat();

        //     // This cannot be 'true'. The program will never exit if it is. This
        //     // lets the robot stop this thread when restarting robot code or
        //     // deploying.
        //     while (!Thread.interrupted()) {
        //         // Tell the CvSink to grab a frame from the camera and put it
        //         // in the source mat. If there is an error notify the output.
        //         if (oi.camera_selection == 0) {
        //             if (cvSink0.grabFrame(mat) == 0) {
        //                 // Send the output the error.
        //                 outputStream.notifyError(cvSink0.getError());
        //                 // skip the rest of the current iteration
        //                 continue;
        //             }
        //         } else {
        //             if (cvSink1.grabFrame(mat) == 0) {
        //                 // Send the output the error.
        //                 outputStream.notifyError(cvSink1.getError());
        //                 // skip the rest of the current iteration
        //                 continue;
        //             }
        //         }

        //         // Put a rectangle on the image
        //         Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
        //         // Give the output stream a new image to display
        //         outputStream.putFrame(mat);
        //     }
        // });

        m_ultraThread = new Thread(() -> {
            try {
                while (!Thread.interrupted()) {
                
                    System.out.println("thread running :) *******************************");
                    ultra.readLastRange();
                    Thread.sleep(100);
                }
            } catch (InterruptedException e) {
                System.out.println("*** Rude. I've been interrupted.");
            }
        });

        // m_visionThread.setDaemon(true);
        // m_visionThread.start();

        m_ultraThread.setDaemon(true);
        m_ultraThread.start();
    }

    private void configureButtonBindings() {

        // buttons
        final JoystickButton climbButton = new JoystickButton(driveStick, 7);
        final JoystickButton cameraButton = new JoystickButton(driveStick, 2);
        final JoystickButton extendHookButton = new JoystickButton(driveStick, 3);
        final JoystickButton retractHookButton = new JoystickButton(driveStick, 5);
        final JoystickButton zoomZoomButton = new JoystickButton(driveStick, 1);

        final JoystickButton positionControlButton = new JoystickButton(otherStick, 3);
        final JoystickButton rotationControlButton = new JoystickButton(otherStick, 4);
        final JoystickButton cancelSpinnerButton = new JoystickButton(otherStick, 2);
        final JoystickButton intakeCellButton = new JoystickButton(otherStick, 5);
        final JoystickButton doorToggleButton = new JoystickButton(otherStick, 6);
        final JoystickButton sensorToggleButton = new JoystickButton(otherStick, 1);

        // buttons to commands
        // climbButton.whileHeld(new Climb(climber));
        cameraButton.whenPressed(new SwapCameraFeed(oi));
        // retractHookButton.whileHeld(new RetractHook(climber));
        // extendHookButton.whileHeld(new ExtendHook(climber));
        zoomZoomButton.whileHeld(new IntakeAll(intakeSystem, belt));

        positionControlButton.whenPressed(new PositionControl(spinner));
        rotationControlButton.whenPressed(new RotationControl(spinner));
        cancelSpinnerButton.whenPressed(new DisableSpinner(spinner));
        intakeCellButton.whileHeld(new SpinCellIn(intakeSystem));
        doorToggleButton.whenPressed(new ToggleDoor(outputSystem));
        sensorToggleButton.whenPressed(new ToggleSensor(spinner));
    }
}
