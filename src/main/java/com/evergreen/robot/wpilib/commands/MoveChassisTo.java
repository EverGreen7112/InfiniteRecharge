package com.evergreen.robot.wpilib.commands;

import com.evergreen.robot.wpilib.Chassis;

import edu.wpi.first.wpilibj2.command.PIDCommand;

/**
 * MoveChassisTo
 */
public class MoveChassisTo extends PIDCommand {

    double m_distance;
//moving chassis to a certin distance with pid
    public MoveChassisTo(double distance) {
        super(Chassis.getInstance().getDistancePID(), 
        () -> Chassis.getInstance().getDistance(), () -> distance, Chassis.getInstance()::move, Chassis.getInstance());
    }

    @Override
    public boolean isFinished() {
        return ((getController().getPositionError())<=Chassis.getInstance().getPIDDistanceTolerance());
    }

    
}