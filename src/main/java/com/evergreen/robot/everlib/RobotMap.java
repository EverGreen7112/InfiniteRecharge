package com.evergreen.robot.everlib;

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
            climbingElevator = 5,
            climbingPuller = 6,
            thrower = 8,
            aimer = 9;

    }
    
    //Detail Piston Components
    public interface PistonPorts {
    
        
    }
    
    //Detail Analog Components
    public interface AnalogPorts {
        
    }
    
    //Detail Digital components
    public interface DigitalPorts {
        public static final int 
        throwerEncouderA =1,
        throwerEncouderB =2;       
    }

    //Declare Encoder
    public interface EncoderPorts {
        public static final int 
            climbingElevatorA = 0,
            climbingElevatorB = 1;
    }
    
    //Detail Joysticks used
    public interface JoystickPorts {
        public static final int
            rightChasisJS = 0,
            leftChassisJS = 1,
            buttonJS = 2;        
    }
    
    //Detail Cameras used
    public interface CameraPorts {
    
        
    }
}
