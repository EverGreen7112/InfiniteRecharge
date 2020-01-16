package com.evergreen.robot;

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
     * CollectorComponents
     * <b> motor </b> - The motor moving the collecting subsystem
     */
    public interface CollectorComponents {
        MotorController motor = new MotorController(ControllerType.TALON_SRX, MotorPorts.collector); //TODO Change to the correct one when decided
    }


    /**
     * SubsystemCComponents
     */
    public interface SubsystemCComponents {

    }
}