package com.evergreen.robot.utils;

/**
 * DoubleArgCommand used for commands that use especially one argument like distance, time, speed etc.
 */
public interface DoubleArgCommand {
    void setValue(double value);

    double getValue();

}