/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OutputSystem extends SubsystemBase {
  /**
   * Creates a new OutputSystem.
   */
  boolean hatchIsClosed;

  public OutputSystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Hatch is Closed:", hatchIsClosed);
  }

  public void openHatch() {
    hatchIsClosed = false;
  }

  public void closeHatch() {
    hatchIsClosed = true;
  }

  public boolean hatchIsClosed() {
    return hatchIsClosed;
  }

}
