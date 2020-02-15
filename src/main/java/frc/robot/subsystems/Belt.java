/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Belt extends SubsystemBase {

  public Talon belt;
  private final double BELT_SPEED = 1.0;

  private Timer timer;

  /**
   * 
   * Creates a new Belt.
   */
  public Belt() {

    timer = new Timer();
    timer.start();
    double start = timer.get();

    belt = new Talon(9);
    belt.setInverted(false);

    System.out.println("Belt.java:Belt():" + Double.toString(timer.get() - start));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double start = timer.get();

    System.out.println("Belt.java:periodic():" + Double.toString(timer.get() - start));
  }

  public void turnBeltCClockwise() {
    belt.set(BELT_SPEED);
  }

  public void turnBeltClockwise() {
    belt.set(-BELT_SPEED);
  }

  public void turnBeltOff() {
    belt.set(0.0);
  }

  public void turnBelt(double speed) {
    System.out.println(speed);
    belt.set(speed);
  }

}
