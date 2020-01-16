package com.evergreen.everlib.subsystems.motors.subsystems;

import java.util.List;
import java.util.Map;

import com.evergreen.everlib.subsystems.SubsystemEG;
import com.evergreen.everlib.subsystems.sensors.DistanceSensor;
import com.evergreen.everlib.shuffleboard.loggables.LoggableData;
import com.evergreen.everlib.shuffleboard.loggables.LoggableDouble;
import com.evergreen.everlib.utils.ranges.Range;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A {@link Subsystem} consisting of one or more motor m_controllers.
 */
public class MotorSubsystem extends SubsystemEG {
    /**The subsystem's motor controllers. */
    protected MotorController[] m_controllers;

    /**The range in which the subsystem is allowed to move. */
    protected Range m_Range;

    /**The sensor mesuring the distance the subsystem has gone. */
    protected DistanceSensor m_distanceSensor;
    
    public MotorSubsystem(String name, MotorController... motors)
    {
        super(name);    
        m_controllers = motors;
        m_Range = (v) -> true;
    }

    public MotorSubsystem(String name, DistanceSensor distanceSensor, MotorController... motors) {
        super(name);
        m_controllers = motors;
        m_distanceSensor = distanceSensor;
    }

    public MotorSubsystem(String name, DistanceSensor sensor, Range range, MotorController... motors)
    {
        super(name);
        m_controllers = motors;
        m_distanceSensor = sensor;
        m_Range = range;
    }

    public MotorSubsystem(String name, DistanceSensor distanceSensor, Range Range, 
        Command defaultCommand, MotorController... motors)
    {
        super(name, defaultCommand);
        m_controllers = motors;
        m_distanceSensor = distanceSensor;
        m_Range = Range;
    }


    public void move(double speed) {
        for (MotorController control : m_controllers) {
            control.set(speed);
        }
    }

    public void set(int index, double speed)
    {
        if(canMove()) 
            m_controllers[index].set(speed);
    }

    public void set(Map<Integer, Double> speedMap)
    {
        if(canMove()) 
            speedMap.forEach(this::set);
    }

    public void stop()
    {
        for(MotorController motor : m_controllers)
        {
            motor.stopMotor();
        }

    }

    public DistanceSensor getSensor() {
        return m_distanceSensor;
    }

    public MotorController[] getMotorControllers() {
        return m_controllers;
    }

    public boolean canMove() {
        return m_Range.inRange(getDistance()) && m_subsystemSwitch.get();
    }

    @Override
    public List<LoggableData> getLoggableData() {
        List<LoggableData> loggables = super.getLoggableData();

        for (int i = 0; i < m_controllers.length; i++) {
            loggables.add(new LoggableDouble(
                getName() + " - Controller #" + i + " speed", m_controllers[i]::get));            
        }

        return super.getLoggableData();
    }

    
}
