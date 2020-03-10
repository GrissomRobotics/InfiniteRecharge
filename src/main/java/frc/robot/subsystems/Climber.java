/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */
  private SpeedController winch = new PWMVictorSPX(12);
  private SpeedController leftHook = new PWMVictorSPX(11);
  private SpeedController rightHook = new PWMVictorSPX(10);

  private final double WINCH_SPEED = 0.5;
  private final double HOOK_SPEED = 0.50;

  public Climber() {
    super();

    winch.setInverted(false);
    leftHook.setInverted(true);
    rightHook.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //winch functions

  public void ascend() {
    winch.set(WINCH_SPEED);
    
  }

  public void descend() {
    winch.set(-WINCH_SPEED);
    
  }
  
  public void stopWinch(){
    winch.set(0.0);
  }

  //hook functions

  public void extend()  {
    rightHook.set(HOOK_SPEED);
    leftHook.set(HOOK_SPEED);
  }

  public void retract() {
    rightHook.set(-HOOK_SPEED);
    leftHook.set(-HOOK_SPEED);
    
  }

  public void extendRightHook(){
    rightHook.set(HOOK_SPEED);
  }

  public void retractRightHook() {
    rightHook.set(-HOOK_SPEED);    
  }

  public void extendLeftHook()  {
    leftHook.set(HOOK_SPEED);
  }

  public void retractLeftHook() {
    leftHook.set(-HOOK_SPEED);
    
  }
  
  public void stopHook(){
    rightHook.set(0.0);
    leftHook.set(0.0);
  } 
}
