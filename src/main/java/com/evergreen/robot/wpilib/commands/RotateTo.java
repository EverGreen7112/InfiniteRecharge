package com.evergreen.robot.wpilib.commands;

import com.evergreen.robot.wpilib.Chassis;

import edu.wpi.first.wpilibj2.command.PIDCommand;

public class RotateTo extends PIDCommand {
//rotating to a certin point with pid
    public RotateTo(double setpoint) {
        
        super(
            Chassis.getInstance().getAnglePID(), //Controller
            () -> Chassis.getInstance().getGyro().getAngle() , //Mesurement Source
            () -> setpoint, //Setpoint Supplier
            Chassis.getInstance()::rotate, //Output Consumer
            Chassis.getInstance()); //Requirement
        
    }

}