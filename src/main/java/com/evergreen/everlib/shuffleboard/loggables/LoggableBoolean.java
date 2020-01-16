package com.evergreen.everlib.shuffleboard.loggables;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * LoggableNumber
 */
public class LoggableBoolean extends LoggableData {
    
    Supplier<Boolean> m_stream;

    public LoggableBoolean(String key, Supplier<Boolean> stream) {
        super(key);
        m_stream = stream;
    }
    
    @Override
    public void addToDashboard() {
        SmartDashboard.putBoolean(getKey(), m_stream.get());
    }

    @Override
    public String getType() {
        return "Boolean";
    }

    @Override
    public String getValue() {
        return m_stream.get().toString();
    }
}