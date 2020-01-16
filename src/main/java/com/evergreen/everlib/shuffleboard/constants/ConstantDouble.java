package com.evergreen.everlib.shuffleboard.constants;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Preferences;


/**
 * LoggableNumber
 */
public class ConstantDouble extends Constant implements Supplier<Double> {
    
    double m_defaultVal;

    public ConstantDouble(String key, double initValue) {
        super(key, initValue);
        m_defaultVal = initValue;
    }

    @Override
    public void addToDashboard() {
        Preferences.getInstance().putDouble(getPath(), m_defaultVal);
    }

    @Override
    public String getType() {
        return "Double";
    }

    public void setValue(double value) {
        m_defaultVal = value;
        addToDashboard();
    }

    @Override
    public Double get() {
        return Preferences.getInstance().getDouble(getPath(), m_defaultVal);
    }

}