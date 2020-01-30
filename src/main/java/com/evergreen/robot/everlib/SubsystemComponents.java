package com.evergreen.robot.everlib;

import com.evergreen.everlib.subsystems.motors.subsystems.MotorController;
import com.evergreen.everlib.subsystems.motors.subsystems.MotorController.ControllerType;

/**
 * SubsystemComponents
 */
public interface SubsystemComponents extends RobotMap {

    /**
     * SubsystemAComponents
     */
    public interface ChassisComponents {
        MotorController chassisLeft = new MotorController(
            new MotorController(ControllerType.TALON_SRX, MotorPorts.chassisLeftFront),
            new MotorController(ControllerType.VICTOR_SPX, MotorPorts.chassisLeftBack));
        MotorController chassisRight = new MotorController(
            new MotorController(ControllerType.TALON_SRX, MotorPorts.chassisRightFront), 
            new MotorController(ControllerType.VICTOR_SPX, MotorPorts.chassisRightBack));
    }

    /**
     * SubsystemBComponents
     */
    public interface SubsystemBComponents {
            
    }


    /**
     * SubsystemCComponents
     */
    public interface SubsystemCComponents {
    
        
    }
}