/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.custom;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Add your docs here.
 */
public class UltrasonicObject {

    private UltrasonicSensor ultra;
    private double ultraValue = -1;
    private Pattern regex = Pattern.compile("R[0-9]{4}"); // match capital R, followed by 4 digits

    public UltrasonicObject(UltrasonicSensor ultrasonicSensor){
        this.ultra = ultrasonicSensor;
    }

    public void readLastRange() {
        String input = ultra.ultrasonicSerialPort.readString(10); // read the most recent 10 characters
        Matcher matcher = regex.matcher(input);
        String s = null; // this will ultimately contain the final range
        while (matcher.find()) {
            s = matcher.group();
        }

        // TODO: implement try catch
        if (s != null) {
            ultraValue = Double.parseDouble(s.substring(1));
        }

        System.out.println("ultra from object: " + ultraValue);
    }

    public double getLastRange(){
        return ultraValue;
    }
    
}
