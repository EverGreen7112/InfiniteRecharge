package com.evergreen.robot.wpilib;

/**
 * RobotMap
 */
public interface RobotMap { 

    public interface MotorPorts {
        public static final int
            thrower = 0,
            aimer = 1;

    
}
   public interface DigitalPorts {
       public static final int
        upperAimerOption = 0,
        lowerAimerOption = 1;

   } 
   public interface AnalogPorts{
       public static final int 
        throwerA = 0,
        throwerB = 1,
        aimerA = 2,     
        aimerB = 3;     
   }
}