package com.evergreen.robot.wpilib;

import java.util.Set;

import com.evergreen.everlib.structure.Tree;
import com.evergreen.robot.RobotMap.ButtonPorts;
import com.evergreen.robot.RobotMap.JoystickPorts;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.evergreen.robot.wpilib.subsystem.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Robot
 */
public class Robot extends TimedRobot {
    boolean m_activate = true;
    boolean m_lifting = false;

    //Creates JS objects
    public Joystick operatorJS = new Joystick(JoystickPorts.operatorJS);

    

    @Override
    public void testInit() {
        // CommandScheduler.getInstance().run();
        // new JoystickButton(operatorJS, 1).whenPressed(Rolletta.getInstance().lift());
        // new JoystickButton(operatorJS, 3).whenPressed(new PrintCommand("Argh"));
        // Chassis.getInstance().drive(0.5, 0.5);
        // Chassis.getInstance().getRightControllerGroup().set(0.7);
        new JoystickButton(operatorJS, 1).whenPressed(new PrintCommand("hello"));
    }

   



    @Override
    public void testPeriodic() {
    //    if(m_activate){
    //        Rolletta.getInstance().lift().schedule();
    //        m_activate =false;
    //    }
        // Rolletta.getInstance().
        // move(Rolletta.getInstance().isLifting());
        // if (Rolletta.getInstance().isUp() != start)
        
        // else Rolletta.getInstance().m_lifter.set(0);
        // SmartDashboard.putBoolean("Up", Rolletta.getInstance().isUp());
        // SmartDashboard.putBoolean("Down", Rolletta.getInstance().isDown());
        // SmartDashboard.putBoolean("lifting", m_lifting);
        // new PrintCommand("hello").schedule();
        // CommandScheduler.getInstance().run();
        CommandBase com =new RunCommand(() -> Chassis .getInstance().drive(0.5, 0.5),Chassis.getInstance());
       com.schedule();
       CommandScheduler.getInstance().run();
       
    }



    @Override
    public void robotPeriodic() {
        new PrintCommand("hey").schedule();
    }
    
    @Override
    public void disabledInit() {
        // new JoystickButton(operatorJS, ButtonPorts.operatorJSA).whenPressed(Rolletta.getInstance().m_calibrateGreen());
        // new JoystickButton(operatorJS, ButtonPorts.operatorJSB).whenPressed(Rolletta.getInstance().m_calibrateRed());
        // new JoystickButton(operatorJS, ButtonPorts.operatorJSX).whenPressed(Rolletta.getInstance().m_calibrateBlue());
        // new JoystickButton(operatorJS, ButtonPorts.operatorJSY).whenPressed(Rolletta.getInstance().m_calibrateYellow()); 
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
    @Override
    public void teleopInit() {
        
    }
    
}   

