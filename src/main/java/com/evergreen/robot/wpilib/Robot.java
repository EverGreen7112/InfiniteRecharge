package com.evergreen.robot.wpilib;

import com.evergreen.everlib.structure.Tree;
import com.evergreen.robot.RobotMap.ButtonPorts;
import com.evergreen.robot.RobotMap.JoystickPorts;

import edu.wpi.first.wpilibj.Joystick;
import com.evergreen.robot.wpilib.subsystem.Shooter;

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
        // TODO Auto-generated method stub

    public void autonomousInit() {
        
    }

    @Override
    protected void bindButtons() {
        new JoystickButton(operatorJS, ButtonPorts.operatorJSA).whenHeld(Climb.getInstance().m_up);
        new JoystickButton(operatorJS, ButtonPorts.operatorJSB).whenHeld(Climb.getInstance().m_pull);
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
    @Override
    public void robotInit() {
        
    }
    @Override
    public void robotPeriodic() {
        Shooter.getInstance().updatePassDistance();
        
    }

    

    
}   

