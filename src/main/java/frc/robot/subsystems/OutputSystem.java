/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OutputSystem extends SubsystemBase {
  /**
   * Creates a new OutputSystem.
   */
  private Servo servoDoor;
  boolean hatchIsClosed;

  private Timer timer;

  public OutputSystem() {
    timer = new Timer();
    timer.start();
    double start = timer.get();

    servoDoor = new Servo(7);

    System.out.println("OutputSystem.java init:" + Double.toString(timer.get() - start));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double start = timer.get();
    
    SmartDashboard.putBoolean("Hatch is Closed:", hatchIsClosed);
    
    System.out.println("Door Subsystem:" + Double.toString(timer.get() - start));
  }

  // might need to swap values

  public void openHatch() {
    servoDoor.set(0);
    hatchIsClosed = false;
  }

  public void closeHatch() {
    servoDoor.set(1);
    hatchIsClosed = true;
  }

  public boolean hatchIsClosed() {
    return hatchIsClosed;
  }

}
