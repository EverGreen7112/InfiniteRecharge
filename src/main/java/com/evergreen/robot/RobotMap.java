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
            chassisLeftBack = 0,
            chassisLeftFront = 1,
            chassisRightBack = 2,
            chassisRightFront = 3,
            collector = 4, //TODO Change port when decided
            climbUp = 5,
            climbPull = 6,
            thrower = 8,
            aimer = 9,
            passer = 10; //The motor which passes a power cell from the storage to the shooter

    }
    
    //Detail Piston Components
    public interface PistonPorts {
    
        
    }
    
    
    //Detail Digital components
    public interface DigitalPorts {
        public static final int 
        throwerEncouderA =1,
        throwerEncouderB =2,
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
