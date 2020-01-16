package com.evergreen.everlib.subsystems.motors.commands;

import java.util.function.Supplier;

import com.evergreen.everlib.utils.PIDSettings;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/**
 * _RunPID
 */
public class _RunPID extends PIDCommand { 

    PIDSettings m_settings;

    public _RunPID(String name, PIDSettings pidSettings, Supplier<Double> target) {
        super(
            pidSettings.getController(),
            pidSettings::getMeasurment,
            () -> target.get(),
            pidSettings::write,
            pidSettings.getSubsystem());

        m_settings = pidSettings;
    }

    @Override
    public void execute() {
        super.execute();
        m_controller.setPID(m_settings.kP(), m_settings.kI(), m_settings.kD());

        m_controller.setTolerance(m_settings.getTolerance());
    }

    @Override
    public boolean isFinished() {
        return m_controller.atSetpoint();
    }

    public PIDSettings getSettings() {
        return m_settings;
    }

    public double getSetpoint() {
        return m_setpoint.getAsDouble();
    }
}