package com.evergreen.everlib.shuffleboard.constants;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Preferences;

/**
 * LoggableNumber
 */
public class ConstantInt extends Constant implements Supplier<Integer> {
    
    int m_defaultVal;

    public ConstantInt(String key, int initValue) {
        super(key, initValue);
        m_defaultVal = initValue;
    }
    
    @Override
    public void addToDashboard() {
        Preferences.getInstance().putInt(getPath(), m_defaultVal);
    }

    @Override
    public String getType() {
        return "Integer";
    }
    public void increaseAbsolute(int increaseBy) {
        m_defaultVal += increaseBy;
        addToDashboard();
    }

    /**
     * Increases the constants value in percentage, according to an input 100%.
     * @param percentage - The percentage to rise
     * @param maxValue - The 100% the percentage are a part of
     */
    public void increasePercentage(int percentage, int maxValue) {
        m_defaultVal += maxValue * (percentage / 100);
        addToDashboard();
    } 

    public void setValue(int newValue) {
        m_defaultVal = newValue;
        reset();
    }

    @Override
    public Integer get() {
        return Preferences.getInstance().getInt(getPath(), m_defaultVal);
    }
}