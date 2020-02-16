package com.evergreen.robot.wpilib;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * DoubleArgCommand used for commands that use especially one argument like distance, time, speed etc. 
 */
public interface DoubleArgCommand extends Command {
public void setValue(double value);
public double getValue();
    
}