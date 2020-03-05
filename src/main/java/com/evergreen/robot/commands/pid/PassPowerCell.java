package com.evergreen.robot.commands.pid;

import com.evergreen.robot.subsystem.Shooter;
import com.evergreen.robot.subsystem.Storage;
import com.evergreen.robot.utils.DoubleArgCommand;
import com.evergreen.robot.utils.Utilites;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * fully (aim, accelerate, passToShooter) pass the power cell to the desierd distance
 */
public  class PassPowerCell extends SequentialCommandGroup implements DoubleArgCommand{
   private _PassPowerCell m_pass;
    /**
     * 
     * @param distance desierd distance for throwing cm 
    */
    public PassPowerCell(double distance) {
        m_pass = new _PassPowerCell(distance);
        
        addCommands(m_pass,Shooter.getInstance().getAimDown(),Utilites.waitForShooting(),Storage.getInstance().passBySensorCmd());
       
    
    }
    

    /**
     * 
     * @param distance desierd distance for throwing cm 
     */
    public void setDistance(double distance) {
        m_pass.setDistance(distance);
    }
    /**
     * 
     * @return desierd distance for throwing cm 
     */
    public double getDistance(){
        return m_pass.getDistance();
    }
    /**
     * 
     * @return desierd distance for throwing cm 
     */
    public double getInstance(){
        return m_pass.getDistance();
    }
    /**
     * @param distance desierd distance for throwing cm 
     */
    @Override
    public void setValue(double distance) {
        setDistance(distance);
        
    }
    /**
     * @return desierd distance for throwing cm 
     */
    @Override
    public double getValue() {
        // TODO Auto-generated method stub
        return getDistance();
    }
}