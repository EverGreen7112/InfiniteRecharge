package com.evergreen.everlib.shuffleboard.loggables;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * LoggableNumber
 */
public class LoggableString extends LoggableData {
    
    Supplier<String> m_stream;

    public LoggableString(String key, Supplier<String> stream) {
        super(key);
        m_stream = stream;
    }

    @Override
    public void addToDashboard() {
        SmartDashboard.putString(getKey(), m_stream.get());
    }

    @Override
    public String getType() {
        return "String";
    }

    @Override
    public String getValue() {
        return m_stream.get().toString();
    }


}