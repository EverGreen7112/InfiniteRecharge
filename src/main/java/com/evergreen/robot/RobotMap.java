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
            collector = 4; //TODO Change port when decided

    }
    
    //Detail Piston Components
    public interface PistonPorts {
    
        
    }
    
    //Detail Analog Components
    public interface AnalogPorts {
        
    }
    
    //Detail Digital components
    public interface DigitalPorts {
        
        
    }

    //Declare Encoder
    public interface EncoderPorts {
        
    }
    
    //Detail Joysticks used
    public interface JoystickPorts {
        public static final int 
            button = 0; //TODO Change port when decided
        
    }
    
    //Detail Cameras used
    public interface CameraPorts {
    
        
    }
}