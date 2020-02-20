package com.evergreen.robot.wpilib;

import java.util.Set;

import com.evergreen.everlib.structure.Tree;
import com.evergreen.robot.RobotMap.ButtonPorts;
import com.evergreen.robot.RobotMap.JoystickPorts;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.evergreen.robot.wpilib.commands.ResetGyro;
import com.evergreen.robot.wpilib.subsystem.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Robot
 */

public class Robot extends TimedRobot {
    boolean m_activate = true;
    boolean m_lifting = false;
    public static Joystick 
        m_leftJoystick = new Joystick(JoystickPorts.leftChassisJS),
        m_righJoystick = new Joystick(JoystickPorts.rightChasisJS),
        m_operatorJoystick = new Joystick(2);

    private static Button buttonX = 
        new JoystickButton(m_operatorJoystick, 1);
    private static Button buttonA = 
        new JoystickButton(m_operatorJoystick, 2);
    private static Button buttonB = 
        new JoystickButton(m_operatorJoystick, 3);
    private static Button buttonY = 
        new JoystickButton(m_operatorJoystick, 4);
    
    //Creates JS objects
    
    public static double getRightJoystick() {
        return m_righJoystick.getY();
    }

    public static double getLeftJoystick() {
        return m_leftJoystick.getY();
    }
    
    
    @Override
    public void testInit() {
        // Rolletta.getInstance().toggle().schedule();
        // new JoystickButton(m_joystick,1).whileHeld(Collector.getInstance().collectCmd());
        // new JoystickButton(m_joystick, 3).whileHeld(Collector.getInstance().collectCmd(-0.7));
        // Collector.getInstance().collectCmd().schedule();
        // Collector.getInstance().collectCmd().schedule();
        // CommandScheduler.getInstance().run();

        // System.out.println("PRINTING WHY");
        // Command cmd  
        CommandScheduler.getInstance().run();
    }

    @Override
    public void testPeriodic() {
        // new PrintCommand("message").schedule();
    }
    
    
    
    @Override
    public void robotPeriodic() {
        // new PrintCommand("CHECK").schedule();
        CommandScheduler.getInstance().run();
        // System.out.println("TEST");

        System.out.println("REFLECTIVE CENTER " +
            SmartDashboard.getNumber("Distance", -2));
    }
    

    
    @Override
    public void disabledInit() {
       
    }
    
    @Override
    public void disabledPeriodic() {
        // CommandScheduler.getInstance().run();
    
    }

    @Override
    public void robotInit() {
       CommandScheduler.getInstance().registerSubsystem(
           Shooter.getInstance(),
           Chassis.getInstance(),
           Climb.getInstance(),
           Collector.getInstance(),
           Rolletta.getInstance(),
           Storage.getInstance()
       );
        
    }
    
    @Override
    public void autonomousInit() {
        Chassis.getInstance().move(0.3);
        try {
            Thread.sleep(1500);
          } catch (InterruptedException e) {
            e.printStackTrace();
            throw new RuntimeException();
          }
          Chassis.getInstance().move(0);
    }
    
    @Override
    public void autonomousPeriodic() {
        // CommandScheduler.getInstance().run();

        
    }
    

    @Override
    public void teleopPeriodic() {
        // CommandScheduler.getInstance().run();
        
    }
      
    @Override
    public void teleopInit() {
        Chassis.getInstance().getDefaultDrive().schedule();//checed
        // new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSLT).whileHeld(Shooter.getInstance().getAccelerateToThrow());
        // new JoystickButton(m_operatorJoystick,ButtonPorts.operatorJSRT).whileHeld(Storage.getInstance().getPass());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSLB).whileHeld(Climb.getInstance().m_up());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSRB).whileHeld(Climb.getInstance().getClimbDown());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSB).whileHeld(Climb.getInstance().m_pull());
        new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSRS).whenPressed(Rolletta.getInstance().toggle());
        // new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSStart).whenPressed(Rolletta.getInstance().getRotationControl());
        // new JoystickButton(m_operatorJoystick, ButtonPorts.operatorJSBack).whenPressed(Rolletta.getInstance().getPositionControl());
        //TODO: add climb down on RB
        
       
    };
    
}   

