package com.evergreen.robot.wpilib;

import com.evergreen.everlib.structure.Tree;
import com.evergreen.robot.RobotMap.ButtonPorts;
import com.evergreen.robot.RobotMap.JoystickPorts;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.TimedRobot;
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
        //The robot elevates the hook while the button is held
        new JoystickButton(operatorJS, ButtonPorts.operatorJSA).whenHeld(Climb.getInstance().m_up);
        //The robot pulls itself upwards while the button is held
        new JoystickButton(operatorJS, ButtonPorts.operatorJSB).whenHeld(Climb.getInstance().m_pull);

        //The robot collects Power Cells while the button is held
        new JoystickButton(operatorJS, ButtonPorts.operatorJSX).whenHeld(Collector.getInstance().collect);

        //The robot passes a Power Cells to the Shooter when the button is pressed
        new JoystickButton(operatorJS, ButtonPorts.operatorJSY).whenPressed(Storage.getInstance().passByTime);
        //TODO check which passing command is the best - by time or by sensor

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

    

    
}   

