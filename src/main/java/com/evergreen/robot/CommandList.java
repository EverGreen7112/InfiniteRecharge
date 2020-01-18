package com.evergreen.robot;

import com.evergreen.everlib.oi.joysticks.JoystickEG;
import com.evergreen.everlib.subsystems.motors.commands.MoveMotorSystem;

/**
 * The list of all commands used by the robot, for both {@link OI} configurations
 * this shoulf be seperated into
 */
public interface CommandList extends SubsystemConstants {


    public interface SubsystemACommands {
    
    }

    public interface CollectorCommands {
        MoveMotorSystem collect = new MoveMotorSystem("collect", Robot.collector, CollectorConstants.speed);
    }

    public interface SubsystemCCommands {
            
    }

}