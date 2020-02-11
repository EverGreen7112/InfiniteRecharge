package com.evergreen.robot.everlib;

import com.evergreen.everlib.subsystems.motors.commands.MoveMotorSystem;
import com.evergreen.robot.everlib.commands.AimShooter;
import com.evergreen.robot.everlib.commands.AimShooter.AimOption;

/**
 * The list of all commands used by the robot, for both {@link OI} configurations
 * this should be seperated into
 */
public interface CommandList extends SubsystemConstants {

    public interface SubsystemACommands {
    
    }

    public interface CollectorCommands {
        MoveMotorSystem collect = new MoveMotorSystem("collect", Robot.collector, CollectorConstants.speed);
    }

    public interface SubsystemCCommands {
            
    }
    
    public interface ShooterCommands{
        //TODO: ask drivers which aim option to use
        AimShooter aimShooter = new AimShooter("aimShooter", AimOption.TOGGLE);
        
        //TODO: add PID command for the thrower- shooting up,shooting down and to pass to other robots.
        //shooting to target could replace shooting up and shooting down with one command, 
        //for shooting to target PIDSetting should have constructur that get suppliers instead of values.
    }

    public interface ClimbingCommands {
        MoveMotorSystem moveElevator = 
            new MoveMotorSystem("Elevator Moves", Robot.climbElevator, () -> Robot.joystickButton.getY());
        
        MoveMotorSystem pullUp = 
            new MoveMotorSystem("Pull Up", Robot.climbPuller, ClimbingConstants.pullerSpeed);
    }

}