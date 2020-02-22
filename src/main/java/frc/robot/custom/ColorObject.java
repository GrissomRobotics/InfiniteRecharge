/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.custom;

/**
 * Add your docs here.
 */

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorMatchResult;

public class ColorObject {

    private final ColorSensorV3 colorSensor;// = new ColorSensorV3(I2C.Port.kOnboard);
    private Color detectedColor;

    public ColorObject(ColorSensorV3 colorSensor) {

        this.colorSensor = colorSensor;

    }

    public void readLastColor(){
        detectedColor = colorSensor.getColor();

    }

    public Color getColor() {
        return detectedColor;
    }
}
