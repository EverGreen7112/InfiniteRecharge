package com.evergreen.robot.everlib;

import com.evergreen.robot.*;
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
            new MotorController(ControllerType.TALON_SRX, MotorPorts.climbUp);
        MotorController puller = 
            new MotorController(ControllerType.TALON_SRX, MotorPorts.climbPull);
        EncoderEG elevatorEncoder = 
            new EncoderEG(EncoderPorts.climbingElevatorA, EncoderPorts.climbingElevatorB);
    } // TODO: check to which controller types are the actual motors connected.
    /**
     * The Shooter is a subsystem which shoot power cells,
     * it containing two subsystem -the thrower and the aimer.
     */
    public interface ShooterComponents{
        /**
         * The power cells thrower under the Shooter subsystem
         */
        public interface Thrower{
            //TODO: change to the correct type
            MotorController thrower = new MotorController(ControllerType.TALON_SRX, RobotMap.MotorPorts.thrower);
            EncoderEG throwerEncoder = new EncoderEG(DigitalPorts.throwerEncoderA,DigitalPorts.throwerEncoderB);
        }
        /**
         * the shooter subsystem aimer under the Shooter subsystem
         */
        public interface Aimer{
            //TODO: change to the correct type
            MotorController aimer = new MotorController(ControllerType.TALON_SRX, RobotMap.MotorPorts.aimer);
        }
    }
}