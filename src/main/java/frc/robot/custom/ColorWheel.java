package frc.robot.custom;

/**
 * @author Jack Smalligan on behalf of Grissom Robotics
 * @version 1.0 This class represents the Infinite Recharge Color Wheel
 * 
 *          The wheel is laid out in 8 slices, yellow, cyan, green, red, yellow,
 *          cyan, green, red, arranged in that order
 */

public class ColorWheel {
    private double radians;

    /**
     * Construct a color wheel, beginning with no rotation having occurred
     */
    public ColorWheel() {
        radians = 0;
    }

    /**
     * Get the amount that the wheel has been rotated, in radians
     * @return the amount of rotation (rad)
     */
    public double getRadians() {
        return radians;
    }

    /**
     * Get the amount that the wheel has been rotated, in revolutions
     * @return the amount of rotation (revolutions)
     */
    public double getRevolutions() {
        return radians / (2 * Math.PI);
    }

    /**
     * Advance the wheel by one color, which is one eighth of a revolution 
     */
    public void advanceOneColor() {
        radians += Math.PI / 4;
    }
}