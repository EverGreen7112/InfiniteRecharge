package com.evergreen.robot.wpilib.commands;

import com.evergreen.robot.wpilib.Utilites;
import com.evergreen.robot.wpilib.subsystem.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * fully (aim, accelerate, passToShooter) pass the power cell to the desierd distance
 */
public class PassPowerCell extends ParallelCommandGroup {
    _PassPowerCell m_pass;
    /**
     * 
     * @param distance desierd distance in cm
     */
    public PassPowerCell(double distance) {
        m_pass = new _PassPowerCell(distance);
        addCommands(Utilites.toFullShootingCommand(
            new ParallelCommandGroup(Shooter.getInstance().getAimDown(),m_pass)
        ));
        
    }
    public void setDistance(double distance) {
        m_pass.setDistance(distance);
    }
    public double getInstance(){
        return m_pass.getDistance();
    }
}