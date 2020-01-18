package com.evergreen.robot;

import com.evergreen.everlib.subsystems.motors.subsystems.MotorSubsystem;
import com.evergreen.robot.commands.Climb;
import com.evergreen.everlib.subsystems.motors.commands.MoveMotorSystem;
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

    public interface ClimbingCommands {
        MoveMotorSystem moveElevator = 
            new MoveMotorSystem("Elevator Moves", Robot.climbElevator, () -> Robot.joystickButton.getY());
        
        MoveMotorSystem pullUp = 
            new MoveMotorSystem("Pull Up", Robot.climbPuller, ClimbingConstants.pullerSpeed);
    }

}