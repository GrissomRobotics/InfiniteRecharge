/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PWMVictorSPX;

/**
 * Add your docs here.
 */
public class RobotMap {
    //spinner
    public static PWMVictorSPX spinnerWheel;
    //constants
    public static final double SPINNER_WHEEL_SPEED = 0.5;

    public static void init(){

        //spinner subsystem
        spinnerWheel = new PWMVictorSPX(3);

        


    }
}

