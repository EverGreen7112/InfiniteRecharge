package com.evergreen.robot.commands.chassisutils;

import com.evergreen.robot.subsystem.Chassis;
import com.evergreen.robot.utils.DoubleArgCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
//Change m_distance
/**
 * MoveChassisTo
 */
public class MoveChassisTo extends PIDCommand 
//implements DoubleArgCommand 
{
    // private int m_timesInSetPoint = 0;
    // private final int SETPOINT_TIMES_FINISH = 5;
    private double m_distance;
    // public void setDistance(double distance){
    //     m_setpoint = () -> (distance);
    //     m_distance = distance;
    // }
    // public double getDistance(){
    //     return m_distance;
    // }

    // @Override
    // public void execute() {
    //     super.execute();
    //     if (getController().atSetpoint()) m_timesInSetPoint++;
    // }
    
    // //moving chassis to a certin distance with pid
    public MoveChassisTo(double distance) {
        super(
            Chassis.getInstance().getDistancePID(), 
            () -> Chassis.getInstance().getDistance(), 
            () -> distance, 
            Chassis.getInstance()::move, 
            Chassis.getInstance()
        );
        m_distance = distance;
    }
    @Override
    public void initialize() {
        Chassis.getInstance().getRightEncoder().reset();
    }
    @Override
    public void execute() {
        super.execute();
        System.out.println("execute");

    }
    @Override
    public boolean isFinished() {
        boolean res =  (
            getController().atSetpoint() 
        );
        return res;

    }
    @Override
    public void end(boolean interrupted) {
        Chassis.getInstance().move(0);
    }
    //     SmartDashboard.putBoolean("AtSetpoint", res);

    //     return res;

    // }

    // @Override
    // public void end(boolean interrupted) {
    //     Chassis.getInstance().drive(0, 0);
    // }
    // @Override
    // public void setValue(double distance) {
    //    setDistance(distance);
    // }

    // @Override
    // public double getValue() {
    //     return getDistance();
    // }
    
    
    
}