package com.evergreen.robot;

import com.evergreen.everlib.subsystems.motors.subsystems.MotorController;
import com.evergreen.everlib.subsystems.motors.subsystems.MotorController.ControllerType;
import com.evergreen.everlib.subsystems.sensors.EncoderEG;

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
    // TODO: add chassis motors so there will be 3 on each side.
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
    /**
     * The climbing mechanism components: 
     * <ul>
     * <li>Elevator Motor
     * <li>Puller Motor
     */
    public interface ClimbingComponents {
        MotorController elevator = 
            new MotorController(ControllerType.TALON_SRX, MotorPorts.climbingElevator);
        MotorController puller = 
            new MotorController(ControllerType.TALON_SRX, MotorPorts.climbingPuller);
        EncoderEG elevatorEncoder = 
            new EncoderEG(EncoderPorts.climbingElevatorA, EncoderPorts.climbingElevatorB);
    } // TODO: check to which controller types are the actual motors connected.
}