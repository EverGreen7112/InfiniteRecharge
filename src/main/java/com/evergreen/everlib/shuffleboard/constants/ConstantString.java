package com.evergreen.everlib.shuffleboard.constants;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Preferences;

/**
 * LoggableNumber
 */
public class ConstantString extends Constant implements Supplier<String> {
    
    String m_defaultVal;

    public ConstantString(String name, String initValue) {
        super(name, initValue);
        m_defaultVal = initValue;
    }
    
    @Override
    public String getType() {
        return "String";
    }

    @Override
    public void addToDashboard() {
        Preferences.getInstance().putString(getPath(), m_defaultVal);
    }

    @Override
    public String get() {
        return Preferences.getInstance().getString(getPath(), m_defaultVal);
    }


}