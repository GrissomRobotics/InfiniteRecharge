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
  private SpeedController winch;
  private SpeedController hook;
  private final double WINCH_SPEED = 0.75;
  private final double HOOK_SPEED = 0.75;

  private Timer timer;

  public Climber() {
    double start = timer.get();

    winch = new PWMVictorSPX(9);
    hook  = new PWMVictorSPX(8);

    winch.setInverted(false);
    hook.setInverted(false);

    timer = new Timer();
    timer.start();

    System.out.println("Climber.java init:" + Double.toString(timer.get() - start));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double start = timer.get();

    System.out.println("Climber Subsystem:" + Double.toString(timer.get() - start));
  }

  public void ascend() {
    winch.set(WINCH_SPEED);
    
  }

  public void descend() {
    winch.set(-WINCH_SPEED);
    
  }

  public void extend()  {
    hook.set(HOOK_SPEED);
  }

  public void retract() {
    hook.set(-HOOK_SPEED);
    
  }

  public void stopWinch(){
    winch.set(0.0);
  }

  public void stopHook(){
    hook.set(0.0);
  }

  
}

