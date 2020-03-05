package com.evergreen.robot.commands.chassisutils;

/**
 * DriveToNoStop drive without stop while is.
 */
public class DriveToNoStop extends MoveChassisTo {

    public DriveToNoStop(double distance) {
        super(distance);
    }

    @Override
    public void end(boolean interrupted) {
        
    }
    
}