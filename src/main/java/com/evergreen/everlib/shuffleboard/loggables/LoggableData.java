package com.evergreen.everlib.shuffleboard.loggables;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Loggable
 */
public abstract class LoggableData {
    private String m_key;

    public LoggableData(String key) {
        m_key = key;
    }

    public String getKey() {
        return m_key;
    }

    public void setKey(String key) {
        m_key = key;
    }

    
    public abstract void addToDashboard();

    public void remove() {
        SmartDashboard.delete(m_key);
    }

    public abstract String getType();
    public abstract String getValue();
}
