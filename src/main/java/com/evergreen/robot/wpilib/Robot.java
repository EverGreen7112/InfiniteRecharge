package com.evergreen.robot.wpilib;

import com.evergreen.everlib.structure.Tree;
import com.evergreen.robot.RobotMap.ButtonPorts;
import com.evergreen.robot.RobotMap.JoystickPorts;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.evergreen.robot.wpilib.subsystem.Shooter;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Robot
 */
public class Robot extends Tree {
    
    boolean m_lifting = false;

    //Creates JS objects
    public Joystick operatorJS = new Joystick(JoystickPorts.operatorJS);

    

    @Override
    protected void autoConfig() {
        Autonomous.getInstance().schedule();
    }

    @Override
    protected void bindButtons() {
        //The robot elevates the hook while the button is held
        new JoystickButton(operatorJS, ButtonPorts.operatorJSA).whenHeld(Climb.getInstance().m_up());
        //The robot pulls itself upwards while the button is held
        new JoystickButton(operatorJS, ButtonPorts.operatorJSB).whenHeld(Climb.getInstance().m_pull());

        //The robot collects Power Cells while the button is held
        new JoystickButton(operatorJS, ButtonPorts.operatorJSX).whenHeld(Collector.getInstance().collect());

        //The robot passes a Power Cells to the Shooter when the button is pressed
        new JoystickButton(operatorJS, ButtonPorts.operatorJSY).whenPressed(Storage.getInstance().passByTime());
        //TODO check which passing command is the best - by time or by sensor

    }

    @Override
    protected void commandConfig() {

    }

    @Override
    protected void componentSetup() {
        Autonomous.getInstance();
    }

    @Override
    protected void log() {

    }

    @Override
    protected void teleopConfig() {

    }

    // boolean start;

    @Override
    protected void test() {
        Rolletta.getInstance().lift().schedule();
    }



    @Override
    public void testPeriodic() {
        // Rolletta.getInstance().
        // move(Rolletta.getInstance().isLifting());
        // if (Rolletta.getInstance().isUp() != start)
        
        // else Rolletta.getInstance().m_lifter.set(0);

        SmartDashboard.putBoolean("Up", Rolletta.getInstance().isUp());
        SmartDashboard.putBoolean("Down", Rolletta.getInstance().isDown());
        SmartDashboard.putBoolean("lifting", m_lifting);
        CommandScheduler.getInstance().run();

    }

    @Override
    protected void whenEnabled() {
        
    }


    @Override
    public void robotPeriodic() {
        Shooter.getInstance().updatePassDistance();
        Autonomous.getInstance().update();
        CommandScheduler.getInstance().run();
    }
    
    
    @Override
    public void disabledInit() {
        new JoystickButton(operatorJS, ButtonPorts.operatorJSA).whenPressed(Rolletta.getInstance().m_calibrateGreen());
        new JoystickButton(operatorJS, ButtonPorts.operatorJSB).whenPressed(Rolletta.getInstance().m_calibrateRed());
        new JoystickButton(operatorJS, ButtonPorts.operatorJSX).whenPressed(Rolletta.getInstance().m_calibrateBlue());
        new JoystickButton(operatorJS, ButtonPorts.operatorJSY).whenPressed(Rolletta.getInstance().m_calibrateYellow()); 
    }
    
    @Override
    public void disabledPeriodic() {
        CommandScheduler.getInstance().run();    
    }
    
    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
    }
    
}   

