package frc.robot.custom;

import java.util.regex.Matcher;
import java.util.regex.Pattern;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.SerialPort;

//Custom implementation of the MB1013 ultrasonic sensor, reading data from the serial port
public class UltrasonicSensor {
    private SerialPort ultrasonicSerialPort;
    private Pattern regex = Pattern.compile("R[0-9]{4}"); // match capital R, followed by 4 digits
    private Timer timer;

    public UltrasonicSensor(SerialPort ultraserial) {
        timer = new Timer();
        timer.start();
        final double start = timer.get();
        ultrasonicSerialPort = ultraserial;
        ultrasonicSerialPort.reset();
        System.out.println("UltrasonicSensor.java:UltrasonicSensor():" + Double.toString(timer.get() - start));
    }

    // reads the range in millimeters
    public double readLastRange() {
        final double start = timer.get();
        String input = ultrasonicSerialPort.readString(10); // read the most recent 10 characters
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
        System.out.println("UltrasonicSensor.java:readLastRange():" + Double.toString(timer.get() - start));
        return distance_mm;
    }
}