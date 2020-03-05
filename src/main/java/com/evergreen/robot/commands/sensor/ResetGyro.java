package com.evergreen.robot.commands.sensor;

import com.evergreen.robot.subsystem.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * ResetGyro reset the gyro
 */
public class ResetGyro extends CommandBase {
@Override
public void initialize() {
    Chassis.getInstance().resetGyro();
}
@Override
public boolean isFinished() {
    // TODO Auto-generated method stub
    return true;
}
    
}