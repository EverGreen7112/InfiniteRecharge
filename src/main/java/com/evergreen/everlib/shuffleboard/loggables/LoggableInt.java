package com.evergreen.everlib.shuffleboard.loggables;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * LoggableNumber
 */
public class LoggableInt extends LoggableData {
    
    Supplier<Integer> m_stream;

    public LoggableInt(String key, Supplier<Integer> stream) {
        super(key);
        m_stream = stream;
    }

    @Override
    public void addToDashboard() {
        SmartDashboard.putNumber(getKey(), m_stream.get());
    }

    @Override
    public String getType() {
        return "Integer";
    }

    @Override
    public String getValue() {
        return m_stream.get().toString();
    }


}