/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DriverStation;

public class Spinner extends SubsystemBase {

  private PWMVictorSPX spinnerWheel = RobotMap.spinnerWheel;
  public static final ColorSensorV3 colorSensor = RobotMap.m_colorSensor;
  public static ColorMatch colorMatcher;
  private static Color kBlueTarget;
  private static Color kGreenTarget;
  private static Color kRedTarget;
  private static Color kYellowTarget;
  private static Color detectedColor;
  private static Color targetColor;

  /**
   * Creates a new Spinner.
   */
  public Spinner() {
    kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

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
        System.out.println("unexpected color");
        break;
      }
    } else {
      System.out.println("No color data");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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

  public boolean colorIsMatched() {
    detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    boolean colorIsMatched = match.color == targetColor;
    return colorIsMatched;
  }

  public String getColorString() {
    detectedColor = colorSensor.getColor();
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
    return colorString;
  }
  public Color getColor() {
    detectedColor = colorSensor.getColor();
    return detectedColor;
  }


}