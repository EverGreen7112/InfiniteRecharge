package com.evergreen.robot.wpilib.commands;

import com.evergreen.robot.wpilib.Chassis;
import com.evergreen.robot.wpilib.DoubleArgCommand;

import edu.wpi.first.wpilibj2.command.PIDCommand;

public   class RotateTo extends PIDCommand implements DoubleArgCommand {
//rotating to a certin point with pid
    //TODO: check if when we put negtive value it rotate left /right, opsite from positive values.
    private double m_startedGyroPosition ;
    public void setSetPoint(double setpoint){
        m_setpoint =()->setpoint;
    }
    public double getSetPoint(){
        return m_setpoint.getAsDouble();
    }
    public RotateTo(double setpoint) {
        
        super(
            Chassis.getInstance().getAnglePID(), //Controller
            () -> Chassis.getInstance().getOriginalAngle()  , //Mesurement Source
            setpoint, //Setpoint
            Chassis.getInstance()::rotate, //Output Consumer
            Chassis.getInstance()); //Requirement
            
        
    }

    @Override
    public void setValue(double setpoint) {
        setSetPoint(setpoint);

    }

    @Override
    public double getValue() {
        return getSetPoint();
    }
    @Override
    public void schedule() {
        final double setpoint2 = Chassis.getInstance().getAbsuluteAngle() + getSetPoint();
        m_setpoint = () -> setpoint2;
        super.schedule();
    }
    @Override
    public void schedule(boolean interruptible) {
        final double setpoint2 = Chassis.getInstance().getAbsuluteAngle() ;
        m_setpoint = () -> setpoint2;
        super.schedule(interruptible);
    }

}