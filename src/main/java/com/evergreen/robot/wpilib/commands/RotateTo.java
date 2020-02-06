package com.evergreen.robot.wpilib.commands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.evergreen.robot.wpilib.Chassis;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class RotateTo extends PIDCommand {

    public RotateTo(double setpoint) {
        
        super(
            Chassis.getInstance().getAnglePID(), //Controller
            () -> Chassis.getInstance().getGyro().getAngle() , //Mesurement Source
            () -> setpoint, //Setpoint Supplier
            Chassis.getInstance()::rotate, //Output Consumer
            Chassis.getInstance()); //Requirement
        
    }

}