package com.evergreen.robot.wpilib;

import com.evergreen.everlib.structure.Tree;
import com.evergreen.robot.RobotMap.JoystickPorts;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * Robot
 */
public class Robot extends Tree {

    //Creates JS objects
    public Joystick buttonJS = new Joystick(JoystickPorts.buttonJS);

    @Override
    protected void autoConfig() {
        // TODO Auto-generated method stub

    }

    @Override
    protected void bindButtons() {


    }

    @Override
    protected void commandConfig() {
        // TODO Auto-generated method stub

    }

    @Override
    protected void componentSetup() {
        // TODO Auto-generated method stub

    }

    @Override
    protected void log() {
        // TODO Auto-generated method stub

    }

    @Override
    protected void teleopConfig() {
        // TODO Auto-generated method stub

    }

    @Override
    protected void test() {
        // TODO Auto-generated method stub

    }

    @Override
    protected void whenEnabled() {
        // TODO Auto-generated method stub

    }

    

    
}   

