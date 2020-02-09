package com.evergreen.robot.wpilib;

import com.evergreen.robot.wpilib.subsystem.Shooter;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * Robot
 */
public class Robot extends TimedRobot {
    
    @Override
    public void autonomousInit() {
        
    }
    @Override
    public void autonomousPeriodic() {
        
    }
    @Override
    public void teleopInit() {
        
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
        Shooter.getInstance().updatePassDistance();
        
    }

    

    
}   

