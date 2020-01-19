package com.evergreen.robot;

/**
 * A map mapping all the robot's elecronic component into integer ports.
 * <p>
 * 
 */
public interface RobotMap {

    //Detail Motor Components
    public interface MotorPorts {
        public static final int 
            chassisLeftBack = 0,
            chassisLeftFront = 1,
            chassisRightBack = 2,
            chassisRightFront = 3,
            thrower = 4,
            aimer =5;        

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
        
    }
    
    //Detail Joysticks used
    public interface JoystickPorts {
        
        
    }
    
    //Detail Cameras used
    public interface CameraPorts {
    
        
    }
}