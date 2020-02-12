package com.evergreen.robot.wpilib.commands;

import java.lang.module.ModuleDescriptor.Requires;

import com.evergreen.robot.wpilib.Chassis;
import com.evergreen.robot.wpilib.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveWithJoystick extends CommandBase{

    public MoveWithJoystick(){
        
    addRequirements(Chassis.getInstance()); 
        
    }
    @Override
    public void execute() {
      Chassis.getInstance().drive(Robot.getLeftJoystick().getY(),Robot.getRightJoystick().getY());  
    }
    public static MoveWithJoystick getInstance(){
        return new MoveWithJoystick();
    }
}