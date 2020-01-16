package com.evergreen.everlib.shuffleboard.loggables;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * LoggableNumber
 */
public class LoggableDouble extends LoggableData {
    
    Supplier<Double> m_stream;

    public LoggableDouble(String key, Supplier<Double> stream) {
        super(key);
        m_stream = stream;
    }
    
    @Override
    public void addToDashboard() {
        SmartDashboard.putNumber(getKey(), m_stream.get());
    }

    @Override
    public String getType() {
        return "Double";
    }

    @Override
    public String getValue() {
        return m_stream.get().toString();
    }

}