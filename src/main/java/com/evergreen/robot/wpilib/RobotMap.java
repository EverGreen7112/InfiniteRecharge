package com.evergreen.robot.wpilib;

/**
 * RobotMap
 */
public interface RobotMap {
    public interface MotorPorts{
        public static final int 
            chassisLeftBack = 0,
            chassisLeftFront = 1,
            chassisLeftUp=2,
            chassisRightBack = 3,
            chassisRightFront = 4,
            chassisRightUp=5;
    }
    public interface JoystickPorts{
        public static final int 
        joystickLeft=0,
        joystickRight =1;
    }
}