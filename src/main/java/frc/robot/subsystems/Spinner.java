/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import frc.robot.RobotMap;

public class Spinner extends SubsystemBase {

  private PWMVictorSPX spinnerWheel = RobotMap.spinnerWheel;

  /**
   * Creates a new Spinner.
   */
  public Spinner() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinPanelClockwise(){
     spinnerWheel.set(RobotMap.SPINNER_WHEEL_SPEED);
   }

  public void spinPanelCClockwise(){
    spinnerWheel.set(-RobotMap.SPINNER_WHEEL_SPEED);
  }

  public void stopSpinner(){
     spinnerWheel.set(0.0);
  }
}