/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PWMVictorSPX;

/**
 * Add your docs here.
 */
public class RobotMap {
    // spinner
    public static PWMVictorSPX spinnerWheel;

    // color detection stuff
    private static final I2C.Port i2cPort = I2C.Port.kOnboard;

    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a parameter.
     * The device will be automatically initialized with default parameters.
     */
    public static final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    // constants
    public static final double SPINNER_WHEEL_SPEED = 0.5;

    public static void init() {
        // spinner subsystem
        spinnerWheel = new PWMVictorSPX(3);
    }
}
