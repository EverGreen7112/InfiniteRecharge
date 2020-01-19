package com.evergreen.robot;

import java.util.ResourceBundle.Control;

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
            EncoderEG throwerEncouder = new EncoderEG(DigitalPorts.throwerEncouderA,DigitalPorts.throwerEncouderB);
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