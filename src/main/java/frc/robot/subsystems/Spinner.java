/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotMap;
import frc.robot.custom.ColorWheel;
import frc.robot.custom.PollingList;

public class Spinner extends SubsystemBase {

  private PWMVictorSPX spinnerWheel = RobotMap.spinnerWheel;
  private ColorSensorV3 colorSensor = RobotMap.m_colorSensor;
  private ColorWheel wheel;
  private static Color lastColor;
  private boolean colorChanged;
  private PollingList colorData;

  // TODO: figure out what these values should be
  private static int POLLING_SIZE = 5;
  private static double COLOR_THRESHOLD = 0.8; // 4 out of the last 5 times

  /**
   * Note: Any example colors should be calibrated as the user needs, these are
   * here as a basic example.
   */
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  /**
   * A Rev Color Match object is used to register and detect known colors. This
   * can be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final ColorMatch m_colorMatcher = new ColorMatch();

  /**
   * Creates a new Spinner.
   */
  public Spinner() {
    wheel = new ColorWheel();

    colorData = new PollingList(POLLING_SIZE);
    lastColor = RobotMap.m_colorSensor.getColor();

    // color detecting
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * This method runs when we care about the color sensor data
   */
  public void colorDetection() {
    Color detectedColor = colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

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

    colorChanged = match.color != lastColor;

    colorData.add(colorChanged ? 1.0 : 0.0);

    if (colorData.getAverage() >= COLOR_THRESHOLD) {
      lastColor = match.color;
      wheel.advanceOneColor();
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
    SmartDashboard.putBoolean("Color Match", colorChanged);
    SmartDashboard.putNumber("Wheel revolutions", wheel.getRevolutions());
  }

  public void spinPanelClockwise() {
    spinnerWheel.set(RobotMap.SPINNER_WHEEL_SPEED);
  }

  public void spinPanelCClockwise() {
    spinnerWheel.set(-RobotMap.SPINNER_WHEEL_SPEED);
  }

  public void stopSpinner() {
    spinnerWheel.set(0.0);
  }
}