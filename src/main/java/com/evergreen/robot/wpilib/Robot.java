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
    public Joystick m_joystick =new Joystick(2);

    //Creates JS objects

    

    @Override
    public void testInit() {
        // Rolletta.getInstance().toggle().schedule();
        // new JoystickButton(m_joystick,1).whileHeld(Collector.getInstance().collectCmd());
        // new JoystickButton(m_joystick, 3).whileHeld(Collector.getInstance().collectCmd(-0.7));
        // Collector.getInstance().collectCmd().schedule();;
        Collector.getInstance().collectCmd().schedule();
       
        
        
    }
    
    
    
    
    
    @Override
    public void testPeriodic() {
        // Collector.getInstance().collect(0.5);
        //    Rolletta.getInstance().move(false);
        // Shooter.getInstance().m_aimer.m_motor.set(0.2);
        CommandScheduler.getInstance().run();
    }



    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

    }
      
    
    @Override
    public void disabledInit() {
       
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

