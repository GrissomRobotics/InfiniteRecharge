package frc.robot.custom;

import edu.wpi.first.wpilibj.util.Color;

public class WheelTracker {
    private int num_new_colors = 0;
    private int num_new_colors_threshold;
    private Color last_color;
    private double radians_turned = 0.0;

    public WheelTracker(Color first_color, int threshold) {
        assert threshold >= 1;
        this.last_color = first_color;
        this.num_new_colors_threshold = threshold;
    }

    public void setNewColor(Color new_color) {
        if (new_color == this.last_color) {
            this.num_new_colors++;
        }
        if (this.num_new_colors >= this.num_new_colors_threshold) {
            this.radians_turned += Math.PI / 4.0;
            this.last_color = new_color;
            this.num_new_colors = 0;
        }
    }

    public double getRadiansTurned(){
        return this.radians_turned;
    }

    public double getNumRotations() {
        return this.radians_turned / (2.0 * Math.PI);
    }
}
