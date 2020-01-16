package com.evergreen.everlib.shuffleboard.constants;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Preferences;

/**
 * LoggableNumber
 */
public class ConstantBoolean extends Constant implements Supplier<Boolean> {
    
    boolean m_defaultVal;

    public ConstantBoolean(String key, boolean initValue) {
        super(key, initValue);
        m_defaultVal = initValue;
    }

    public ConstantBoolean(String name) {
        this(name, true);
    }

    @Override
    public String getType() {
        return "Boolean";
    }

    @Override
    public Boolean get() {
        return Preferences.getInstance().getBoolean(getPath(), m_defaultVal);
    }

    @Override
    public void addToDashboard() {
        Preferences.getInstance().putBoolean(getPath(), m_defaultVal);
    }

    public void set(boolean value) {
        m_defaultVal = value;
        addToDashboard();
    }

    public void toggle() {
        set(!m_defaultVal);
    }

    public void disable()
    {
        set(false);
    }

    public void enable() 
    {
        set(true);
    }

}