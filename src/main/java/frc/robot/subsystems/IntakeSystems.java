/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeSystems;
public class IntakeSystems extends SubsystemBase {
  /**
   * Creates a new IntakeSystems.
   */
  private final PWMVictorSPX spinningWheel;
  private final PWMVictorSPX armMotor;
  private final DigitalInput upperArmLimit;
  private final DigitalInput lowerArmLimit;

  public IntakeSystems() {
    spinningWheel = new PWMVictorSPX(7);
    armMotor = new PWMVictorSPX(8);
    upperArmLimit = new DigitalInput(0);
    lowerArmLimit = new DigitalInput(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void spinWheelCClockwise(){
    spinningWheel.set(0.4);
  }
  public void spinWheelClockwise(){
    spinningWheel.set(-0.4);
  }
  public void spinWheelOff(){
    spinningWheel.set(0.0);
  }
  public void armMotorCClockwise(){
    armMotor.set(0.4);
  }
  public void armMotorclockwise(){
    armMotor.set(-0.4);
  }
  public void armMotorOff(){
    armMotor.set(0.0);
  }
  public boolean getUpperLimitSwitch(){
    return upperArmLimit.get();
  }
  public boolean getLowerLimitSwitch(){
    return lowerArmLimit.get();
  }
}