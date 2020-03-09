package com.evergreen.robot.utils;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * A map mapping all the robot's elecronic component into integer ports.
 * <p>
 * 
 */
public interface RobotMap {

    //Detail Motor Components
    //every time when you are change port in RobotMap change it in Robot Characterization python file!!!!!!!
    public interface MotorPorts {
        public static final int
        chassisRightBack = 0, //Victor
        chassisRightMiddle = 1, //Talon
        chassisRightFront = 2, //victor
        chassisLeftBack = 3, //Victor
        collector = 4, //Victor
        passer = 5, //Victor. The motor which passes a power cell from the storage to the shooter.
        aimer = 6, //Talon. Aims the shooter.
        lifter = 7, //Victor, Roletta Lifter
        thrower = 8, //Victor
        spinner = 9, //Talon
        climbUp = 12, //Talon
        climbPull = 13, //Victor
        chassisLeftFront = 14, //Victor
        chassisLeftMiddle = 15; //Talon
        
    }
    
    //Detail Piston Components
    public interface PistonPorts {
    
        
    }
    
    
    //Detail Digital components
    public interface DigitalPorts {
        public static final int 
        leftChassisEncoderA = 0,
        leftChassisEncoderB = 1,
        throwerEncoderB = 2,
        throwerEncoderA = 3,
        rolletaMicroSwitchDown = 4,
        rolletaMicroSwitchUp = 5,
        aimerDownSwitch = 6,
        rightEncoderA = 7,
        aimerUpSwitch = 7, //TODO: check AimerUp switch DIO port
        rightEncoderB = 8;
        

    }
    public interface AnalogPorts{
        public static final int 
          collectorUltrasonic = 0,
          storageUltrasonic = 1;

        public static final I2C.Port
            colorSensor = Port.kOnboard;
    }

    
    //Detail Joysticks used
    public interface JoystickPorts {
        public static final int
            rightChasisJS = 0,
            leftChassisJS = 1,
            operatorJS = 2;        
    }

    //Detail the Buttons of each Joystick
    public interface ButtonPorts {
        public static final int 
            operatorJSX = 1,
            operatorJSA = 2,
            operatorJSB = 3,
            operatorJSY = 4,
            operatorJSLB = 5,
            operatorJSRB = 6,
            operatorJSLT = 7,
            operatorJSRT = 8,
            operatorJSBack = 9,
            operatorJSStart = 10,
            operatorJSLeftS = 11,
            operatorJSRS = 12;
    }
    
    //Detail Cameras used
    public interface CameraPorts {
        public static int 
        backCamera = 0;
        
    }
}
