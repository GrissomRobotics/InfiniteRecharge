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
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;

public class Spinner extends SubsystemBase {

  private TalonSRX spinnerWheel;
  private Servo sensorServo;
  private final double SPINNER_WHEEL_SPEED = 0.75;
  private PigeonIMU gyro;
  // was originally public static just incase things go badly now
  public static I2C.Port i2cPort;
  private final ColorSensorV3 colorSensor;
  public static ColorMatch colorMatcher;
  private static Color kBlueTarget;
  private static Color kGreenTarget;
  private static Color kRedTarget;
  private static Color kYellowTarget;
  private static Color detectedColor;
  private static Color targetColor;

  private Timer timer;

  /**
   * Creates a new Spinner.
   */
  public Spinner() {
    timer = new Timer();
    timer.start();
    double start = timer.get();

    spinnerWheel = new TalonSRX(3);
    sensorServo = new Servo(3);
    gyro = new PigeonIMU(spinnerWheel);
    spinnerWheel.setInverted(false);
    /*
     * spinnerWheel.configContinuousCurrentLimit(20,0);
     * spinnerWheel.configPeakCurrentLimit(30,0);
     * spinnerWheel.configPeakCurrentDuration(100,0);
     * spinnerWheel.enableCurrentLimit(true);
     */

    i2cPort = I2C.Port.kOnboard;
    colorSensor = new ColorSensorV3(i2cPort);

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

    System.err.println("****************************************** Spinner.java:Spinner():" + Double.toString(timer.get() - start));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Confidence", match.confidence);
    // SmartDashboard.putString("Detected Color", colorString);
    // SmartDashboard.putBoolean("Color Match", colorMatched);
    double start = timer.get();

    //SmartDashboard.putString("Color: ", getColorString());
    //SmartDashboard.putNumber("Gyro", getGyroData());

    System.out.println("Spinner Subsystem periodic():" + Double.toString(timer.get() - start));
  }

  public void toggleSensor(){
    System.out.println("**************** Running toggleSensor()");
    if(sensorServo.get() > 0.5){
      sensorServo.set(0.0);
    }else{
      sensorServo.set(1.0);
    }

  }

  public void spinManual(double speed) {
    System.out.println("spinner speed: " + speed);
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

  public boolean colorIsMatched() {
    System.out.println("**************** Running colorIsMatched()");
    detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    boolean colorIsMatched = match.color == targetColor;
    return colorIsMatched;
  }

  public String getColorString() {
    System.out.println("**************** Running getColorString()");
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
    System.out.println("**************** Running getColor()");
    detectedColor = colorSensor.getColor();
    return detectedColor;
  }

  public void resetGyro() {
    System.out.println("**************** Running resetGyro()");
    gyro.setFusedHeading(0);
  }

  public double getGyroData() {
    System.out.println("**************** Running getGyroData()");
    return gyro.getFusedHeading();
  }

}