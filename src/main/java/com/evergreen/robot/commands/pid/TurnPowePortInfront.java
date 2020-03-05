package com.evergreen.robot.commands.pid;

import com.evergreen.robot.subsystem.Chassis;
import com.evergreen.robot.subsystem.Shooter;
import com.evergreen.robot.utils.Utilites;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/**
 * TurnPowePortMiddle turn till the robot is infront the power port
 */
public class TurnPowePortInfront extends PIDCommand {
    
    
    public TurnPowePortInfront() {
        super(new PIDController(Chassis.getInstance().getAngleKP(),Chassis.getInstance().getAngleKI(), Chassis.getInstance().getAngleKD()),
         Utilites::getPowerPortToRobotAngle, 0.0, Chassis.getInstance()::rotate, Shooter.getInstance());
        // TODO Auto-generated constructor stub
    }



    
}