package com.evergreen.robot;

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
        chassisLeftBack = 3, //Victor
        chassisLeftMiddle = 15, //Talon
        chassisLeftFront = 14, //Victor
        chassisRightBack = 0, //Victor
        chassisRightMiddle = 1, //Talon
        chassisRightFront = 2, //victor
        collector = 4, //Victor
        climbUp = 8, //Talon
        climbPull = 13, //Victor
        thrower = 12, //Talon
        aimer = 9, //Victor. Aims the shooter.
        passer = 5; //Victor. The motor which passes a power cell from the storage to the shooter.
    }
    
    //Detail Piston Components
    public interface PistonPorts {
    
        
    }
    
    
    //Detail Digital components
    public interface DigitalPorts {
        public static final int 
        throwerEncoderA = 1,
        throwerEncoderB = 2,
        aimerEncoderA = 5,
        aimerEncoderB = 6,
        storageUltrasonicA = 3,
        storageUltrasonicB = 4;       
    }

    //Declare Encoder
    public interface AnalogPorts {
        public static final int 
            climbingElevatorA = 0,
            climbingElevatorB = 1,
            aimerA = 2,
            aimerB = 3,
            throwerA = 4,
            throwerB =5;
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
            operatorJSA = 3,
            operatorJSB = 2,
            operatorJSX = 0,
            operatorJSY = 1;
        
    }
    
    //Detail Cameras used
    public interface CameraPorts {
    
        
    }
}
