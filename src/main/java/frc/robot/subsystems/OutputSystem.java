/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OutputSystem extends SubsystemBase {
  /**
   * Creates a new OutputSystem.
   */
  private DoubleSolenoid doorSolenoid;
  //boolean hatchIsClosed;

  private Timer timer;

  public OutputSystem() {
    timer = new Timer();
    timer.start();
    double start = timer.get();

    doorSolenoid = new DoubleSolenoid(4, 0, 1);

    System.out.println("OutputSystem.java:OutputSystem():" + Double.toString(timer.get() - start));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double start = timer.get();
    
    //SmartDashboard.putBoolean("Hatch is Closed:", hatchIsClosed);
    
    System.out.println("OutputSubsystem.java:periodic():" + Double.toString(timer.get() - start));
  }

  public void openDoor() {
    doorSolenoid.set(Value.kForward);
  }

  public void closeDoor() {
    doorSolenoid.set(Value.kReverse);
  }


  public void toggle() {
    switch(doorSolenoid.get()) {
      case kForward:
        doorSolenoid.set(Value.kReverse);
      case kReverse:
        doorSolenoid.set(Value.kForward);
      case kOff:
        //
    }
  }

}
