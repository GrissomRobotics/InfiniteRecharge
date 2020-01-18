package frc.robot.custom;

import java.util.ArrayList;

/**
 * @author Jack Smalligan for Grissom Robotics
 * @version 1.0
 * @param T the type
 * 
 *          This class stores a set number of samples from a sensor to avoid
 *          false positives.
 */

public class PollingList {

    private ArrayList<Double> data;
    private int size;
    private int addingIndex;

    /**
     * Construct a polling list of the given size
     * 
     * @param size the maximum number of samples to consider
     */
    public PollingList(int size) {
        data = new ArrayList<Double>();
        this.size = size;
        addingIndex = 0;
    }

    /**
     * Add a new element to the list. If there are already size elements, replace
     * the next sample as determined by addingIndex. Otherwise, add to the end
     * 
     * @param element the sample to add
     */
    public void add(Double element) {
        if (data.size() < size) {
            data.add(element);
        } else {
            data.set(addingIndex % size, element);
            addingIndex++;
        }
    }

    /**
     * Determine the average of the samples in the list
     * @return the average of samples in the list
     */
    public double getAverage() {
        double sum = 0;
        for (double d : data) {
            sum += d;
        }
        return sum / data.size();
    }

    public void resetList(){
        for(int i = 0; i < data.size(); i++){
            data.set(i, 0.0);
        }
    }
}