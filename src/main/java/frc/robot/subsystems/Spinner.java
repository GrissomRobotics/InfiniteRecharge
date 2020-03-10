/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import frc.robot.RobotMap;
import frc.robot.custom.ColorObject;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;

public class Spinner extends SubsystemBase {

  // motor stuff

  private final TalonSRX spinnerWheel = new TalonSRX(3);
  private final Servo sensorServo = new Servo(3);
  private final double SPINNER_WHEEL_SPEED = 0.45;

  // color stuff

  private final ColorObject colorObject;
  private final ColorMatch colorMatcher;

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  private Color detectedColor;
  private Color targetColor;

  //private SuppliedValueWidget<Boolean> colorWidget = Shuffleboard.getTab("Cameras").addBoolean("Color", () -> true);

  // gyro stuff

  private PigeonIMU gyro;
  public Thread m_gyroThread;
  private double lastGyroValue;

  public Spinner(ColorObject color) {
    super();

    // spinnerWheel settings
    spinnerWheel.setInverted(false);

    // TODO: Implement???
    // spinnerWheel.configContinuousCurrentLimit(20,0);
    // spinnerWheel.configPeakCurrentLimit(30,0);
    // spinnerWheel.configPeakCurrentDuration(100,0);
    // spinnerWheel.enableCurrentLimit(true);

    // color features
    this.colorObject = color;

    colorMatcher = new ColorMatch();
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);

    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          targetColor = kBlueTarget;
          break;
        case 'G':
          targetColor = kGreenTarget;
          break;
        case 'R':
          targetColor = kRedTarget;
          break;
        case 'Y':
          targetColor = kYellowTarget;
          break;
        default:
          System.out.println("Received unexpected color from game data");
          break;
      }
    } else {
      System.out.println("Could not load game color data");
    }

    // GYRO THREAD
    gyro = new PigeonIMU(spinnerWheel);

    // m_gyroThread = new Thread(() -> {
    //   try {
    //     while (!Thread.interrupted()) {

    //       System.out.println("thread running :) *******************************");
    //       lastGyroValue = gyro.getFusedHeading();
    //       Thread.sleep(100);
    //     }
    //   } catch (InterruptedException e) {
    //     System.out.println("*** Rude. I've been interrupted.");
    //   }
    // });

    // m_gyroThread.setDaemon(true);
    // m_gyroThread.start();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // System.out.println("Color: " + getColorString());
    // System.out.println("Gyro Value: " + getGyroData());

    SmartDashboard.putString("Color: ", getColorString());
    // SmartDashboard.putNumber("Gyro", getGyroData());

    //colorWidget.withProperties(Map.of("colorWhenTrue", getColor()));
  }

  // sensor functions

  public void toggleSensor() {
    if (sensorServo.getAngle() > 45.0) {
      sensorServo.setAngle(0.0);
    } else {
      sensorServo.setAngle(100.0);
    }
  }

  // spinnerWheel functions

  public void spinManual(double speed) {
    spinnerWheel.set(ControlMode.PercentOutput, speed);
  }

  public void spinPanelClockwise() {
    spinnerWheel.set(ControlMode.PercentOutput, SPINNER_WHEEL_SPEED);
  }

  public void spinPanelCClockwise() {
    spinnerWheel.set(ControlMode.PercentOutput, -SPINNER_WHEEL_SPEED);
  }

  public void stopSpinner() {
    spinnerWheel.set(ControlMode.PercentOutput, 0.0);
  }

  // color functions

  public boolean colorIsMatched() {
    System.out.println("**************** Running colorIsMatched()");
    detectedColor = getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    boolean colorIsMatched = match.color == targetColor;
    return colorIsMatched;
  }

  public String getColorString() {
    // System.out.println("**************** Running getColorString()");
    detectedColor = getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    String colorString;
    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }
    System.out.println(colorString);
    return colorString;
  }

  public Color getColor() {
    // System.out.println("**************** Running spinner.getColor()");
    return colorObject.getColor();
  }

  public ColorMatchResult getColorMatch(){
    detectedColor = getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    return match;
  }

  // gyro functions

  public void resetGyro() {
    // System.out.println("**************** Running resetGyro()");
    gyro.setFusedHeading(0);
  }

  public double getGyroData() {
    // System.out.println("**************** Running getGyroData()");
    return lastGyroValue;
  }
}
