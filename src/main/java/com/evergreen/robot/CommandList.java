package com.evergreen.robot;

import com.evergreen.everlib.CommandEG;
import com.evergreen.everlib.shuffleboard.constants.ConstantDouble;
import com.evergreen.everlib.shuffleboard.constants.DashboardConstants;
import com.evergreen.everlib.subsystems.motors.commands.MoveMotorSystem;
import com.evergreen.everlib.subsystems.motors.subsystems.MotorSubsystem;
import com.evergreen.robot.commands.Climb;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * The list of all commands used by the robot, for both {@link OI} configurations
 * this should be seperated into
 */
public interface CommandList extends SubsystemConstants {

    public interface SubsystemACommands {
    
    }

    public interface SubsystemBCommands {
    
        
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