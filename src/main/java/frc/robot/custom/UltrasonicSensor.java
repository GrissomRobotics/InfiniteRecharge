package frc.robot.custom;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.wpi.first.wpilibj.SerialPort;

//Custom implementation of the MB1013 ultrasonic sensor, reading data from the serial port
public class UltrasonicSensor {
    private SerialPort ultrasonicSerialPort;
    private Pattern regex = Pattern.compile("R[0-9]{4}"); // match capital R, followed by 4 digits

    public UltrasonicSensor(SerialPort ultraserial) {
        ultrasonicSerialPort = ultraserial;
        ultrasonicSerialPort.reset();
    }

    // reads the range in millimeters
    public double readLastRange() {
        String input = ultrasonicSerialPort.readString(10); // read the most recent 10 characters
        System.out.println(input);
        Matcher matcher = regex.matcher(input);
        String s = null; // this will ultimately contain the final range
        while (matcher.find()) {
            s = matcher.group();
        }
        double distance_mm = -1;
        
        //TODO: implement try catch
        if (s != null) {
            distance_mm = Double.parseDouble(s.substring(1));
        }
        return distance_mm;
    }
}