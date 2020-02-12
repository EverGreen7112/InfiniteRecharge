package com.evergreen.robot.wpilib.commands;

import com.evergreen.robot.wpilib.subsystem.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * first aim down and then pass the power cell to the desierd distance
 */
public class PassPowerCell extends SequentialCommandGroup {
    _PassPowerCell m_pass;
    /**
     * 
     * @param distance desierd distance in cm
     */
    public PassPowerCell(double distance) {
        m_pass = new _PassPowerCell(distance);
        addCommands(
            Shooter.getInstance().m_aimDown,
            m_pass);
        
    }
    public void setDistance(double distance) {
        m_pass.setDistance(distance);
    }
    public double getInstance(){
        return m_pass.getDistance();
    }
}