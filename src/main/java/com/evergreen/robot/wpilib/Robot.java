package com.evergreen.robot.wpilib;

import com.evergreen.everlib.structure.Tree;
import com.evergreen.robot.RobotMap.ButtonPorts;
import com.evergreen.robot.RobotMap.JoystickPorts;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Robot
 */
public class Robot extends Tree {

    //Creates JS objects
    public Joystick operatorJS = new Joystick(JoystickPorts.operatorJS);

    @Override
    protected void autoConfig() {

    }

    @Override
    protected void bindButtons() {
        new JoystickButton(operatorJS, ButtonPorts.operatorJSA).whenHeld(Climb.getInstance().m_up);
        new JoystickButton(operatorJS, ButtonPorts.operatorJSB).whenHeld(Climb.getInstance().m_pull);
    }

    @Override
    protected void commandConfig() {

    }

    @Override
    protected void componentSetup() {

    }

    @Override
    protected void log() {

    }

    @Override
    protected void teleopConfig() {

    }

    @Override
    protected void test() {

    }

    @Override
    protected void whenEnabled() {
        
    }

    @Override
    protected void update() {
        
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {
        new JoystickButton(operatorJS, ButtonPorts.operatorJSA).whenPressed(Rolletta.getInstance().calibrateGreen);
        new JoystickButton(operatorJS, ButtonPorts.operatorJSB).whenPressed(Rolletta.getInstance().calibrateRed);
        new JoystickButton(operatorJS, ButtonPorts.operatorJSX).whenPressed(Rolletta.getInstance().calibrateBlue);
        new JoystickButton(operatorJS, ButtonPorts.operatorJSY).whenPressed(Rolletta.getInstance().calibrateYellow);
        
    }

    
}   

