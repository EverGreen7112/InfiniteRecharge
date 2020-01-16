package com.evergreen.everlib.subsystems.motors.commands;

import java.util.List;
import java.util.function.Supplier;

import com.evergreen.everlib.CommandEG;
import com.evergreen.everlib.utils.PIDSettings;
import com.evergreen.everlib.shuffleboard.loggables.LoggableData;
import com.evergreen.everlib.shuffleboard.loggables.LoggableDouble;


/**
 * MotorSubsystemPID
 */
public class RunPID extends CommandEG {

    _RunPID m_command;
    
    public RunPID(String name, PIDSettings pidSettings, Supplier<Double> target) {
        super(name);
        m_command = new _RunPID(name, pidSettings, target);
    }

    @Override
    public void schedule() {
        m_command.schedule();
    }

    @Override
    public List<LoggableData> getLoggableData() {
        List<LoggableData> loggables = super.getLoggableData();

        loggables.addAll(List.of(
            new LoggableDouble(getName() + " - kP", m_command.getSettings()::kP),
            new LoggableDouble(getName() + " - kI", m_command.getSettings()::kI),
            new LoggableDouble(getName() + " - kD", m_command.getSettings()::kD),
            new LoggableDouble(getName() + " - kF", m_command.getSettings()::kF),
            new LoggableDouble(getName() + " - Tolerance", m_command.getSettings()::getTolerance),
            new LoggableDouble(getName() + " - Distance", m_command.getSettings()::getMeasurment),
            new LoggableDouble(getName() + " - Period", m_command.getSettings()::getPeriod),
            new LoggableDouble(getName() + " - Setpoint", m_command::getSetpoint),
            
            new LoggableDouble(getName() + " - Error", 
                () -> m_command.getSetpoint() - m_command.getSettings().getMeasurment()),
            new LoggableDouble(getName() + " - Calculated PIDF", () -> 
                m_command.getController().calculate(m_command.getSettings().getMeasurment()))
            ));
        
        return loggables;
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || m_command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        m_command.cancel();
    }


}