/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

public class IntakeSystem extends SubsystemBase {
  /**
   * Creates a new IntakeSystems.
   */
  private final Talon  spinningWheel;
  private final Talon armMotor;

  private final double WHEEL_SPEED = 0.75;
  private final double ARM_SPEED = 0.75;

  private final DigitalInput upperArmLimit;
  private final DigitalInput lowerArmLimit;

  public IntakeSystem() {
    super();

    spinningWheel = new Talon(7);
    armMotor = new Talon(2);

    spinningWheel.setInverted(false);
    armMotor.setInverted(false);

    upperArmLimit = new DigitalInput(0);
    lowerArmLimit = new DigitalInput(1);

    System.out.println("upper:" + getUpperLimitSwitch());
    System.out.println("lower:" + getLowerLimitSwitch());
  }

  // TODO: Toggle Arm :)

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // System.out.println("upper:" + getUpperLimitSwitch());
    // System.out.println("lower:" + getLowerLimitSwitch());

    // SmartDashboard.putBoolean("LowerArmLimit", getLowerLimitSwitch());
    // SmartDashboard.putBoolean("UpperArmLimit", getUpperLimitSwitch());

  }

  // intakeWheel functions

  public void spinWheelCClockwise() {
    spinningWheel.set(WHEEL_SPEED);
  }

  public void spinWheelClockwise() {
    spinningWheel.set(-WHEEL_SPEED);
  }

  public void setSpinWheel(double speed) {
    spinningWheel.set(speed);
  }

  public void spinWheelOff() {
    spinningWheel.set(0.0);
  }

  //arm functions

  public void moveArmMotorDown() {
    armMotor.set(ARM_SPEED);
  }

  public void moveArmUp() {
    armMotor.set(-ARM_SPEED);
  }

  public void stopArmMotor() {
    armMotor.set(0.0);
  }

  public void moveArmMotorManual(double speed) {

    double armSpeed = speed;

    armSpeed = MathUtil.clamp(armSpeed, -ARM_SPEED, ARM_SPEED);
    armMotor.set(armSpeed);

    //TODO: add limit switch logic back in
    // if (getLowerLimitSwitch() && (armSpeed > 0.0)) {
    // System.out.println("Stopping arm due to lower limit switch.");
    // stopArmMotor();
    // } else if (getUpperLimitSwitch() && (armSpeed < 0.0)) {
    // System.out.println("Stopping arm due to upper limit switch.");
    // stopArmMotor();
    // } else {
    // armSpeed = MathUtil.clamp(armSpeed, -ARM_SPEED, ARM_SPEED);
    // armMotor.set(armSpeed);
    // }

  }

  //limit switch

  public boolean getUpperLimitSwitch() {
    return upperArmLimit.get();
  }

  public boolean getLowerLimitSwitch() {
    return lowerArmLimit.get();
  }

}
