package com.evergreen.robot;

import com.evergreen.everlib.subsystems.motors.commands.MoveMotorSystem;
import com.evergreen.everlib.subsystems.motors.commands.RunPID;
import com.evergreen.everlib.utils.InstantCommandEG;
import com.evergreen.robot.ShooterAimer.AimOption;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * The list of all commands used by the robot, for both {@link OI} configurations
 * this shoulf be seperated into
 */
public interface CommandList extends SubsystemConstants {


    public interface SubsystemACommands {
    
    }

    public interface SubsystemBCommands {
    
        
    }

    public interface SubsystemCCommands {
            
    }
    public interface ShooterCommands{
        //TODO: ask drivers which aim option to use
        ShooterAimer aimShooter = new ShooterAimer("aimShooter", AimOption.TOGGLE);
        //TODO: add PID command for the thrower- shooting up,shooting down and to pass to other robots.
        //shooting to target could replace shooting up and shooting down with one command, 
        //for shooting to target PIDSetting should have constructur that get suppliers instead of values.
    }

}