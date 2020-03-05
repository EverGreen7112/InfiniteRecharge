package com.evergreen.robot.commands.sensor;

import com.evergreen.robot.subsystem.Chassis;
import com.evergreen.robot.utils.Utilites;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * ResetOdometry and gyro use when robot at power port 
 */
public class ResetOdometry extends CommandBase{
@Override
public void initialize() {
    Chassis.getInstance().resetGyro();
    Chassis.getInstance().getOdometry().resetPosition(Utilites.POWER_PORT_POSE2D, Chassis.getInstance().getHeading());
}
    
}