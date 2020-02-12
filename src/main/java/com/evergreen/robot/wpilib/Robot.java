package com.evergreen.robot.wpilib;

import com.evergreen.robot.wpilib.RobotMap.JoystickPorts;
import com.evergreen.robot.wpilib.commands.MoveWithJoystick;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * Robot
 */
public class Robot extends TimedRobot {
    static Joystick leftJoystick = new Joystick(JoystickPorts.joystickLeft);
    static Joystick rightJoystick = new Joystick(JoystickPorts.joystickRight);

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        Chassis.getInstance().setDefaultCommand(MoveWithJoystick.getInstance());
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void robotInit() {

    }

    @Override
    public void robotPeriodic() {

    }

    public static Joystick getLeftJoystick() {
        return leftJoystick;
    }

    public static Joystick getRightJoystick() {
        return rightJoystick;
    }
    

    
}   

