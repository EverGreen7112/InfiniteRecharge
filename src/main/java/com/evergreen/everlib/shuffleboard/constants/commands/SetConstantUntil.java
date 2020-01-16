package com.evergreen.everlib.shuffleboard.constants.commands;

import java.util.function.Supplier;

import com.evergreen.everlib.CommandEG;
import com.evergreen.everlib.shuffleboard.constants.ConstantDouble;

/**
 * A command to set a {@link ConstantDouble} a certain value,
 * and reverts it back once a specified condition is met,
 * or once the command is intterupted.
 * 
 * @author Atai Ambus
 */
public class SetConstantUntil extends CommandEG {
    /**The {@link #isFinished()} condition.*/
    Supplier<Boolean> m_until;
    /**The constant to set.*/
    ConstantDouble m_constant;
    /**The value to revert the cosntant back to*/
    double m_initValue;
    /**The value to set the constant to*/
    double m_valueToSet;

    /**
     * Constructs a new {@link SetConstantUntil} command, setting an input {@link ConstantDouble}
     * an input value and reverting it once a input condition is met.
     * 
     * @param name - The command's name, for switch and logging
     * @param constant - The constant to set
     * @param value - The value to set the constant to
     * @param until - The condition under which to revert the constant back
     */
    public SetConstantUntil(String name, 
        ConstantDouble constant, Supplier<Double> value, Supplier<Boolean> until) {
        super(name);
        m_constant = constant;
        m_initValue = constant.get();
        m_valueToSet = value.get();
    }

    /**
     * Constructs a new {@link SetConstantUntil} command, setting an input {@link ConstantDouble}
     * an input value and reverting it once the command is intterupted.
     * 
     * @param name - The command's name, for switch and logging
     * @param constant - The constant to set
     * @param value - The value to set the constant to
     */
    public SetConstantUntil(String name, ConstantDouble constant, Supplier<Double> value) {
        this(name, constant, value, () -> false);
    }

    /**When the command starts - set the command the input value */
    @Override
    public void initialize() {
        m_constant.setValue(m_valueToSet);
    }

    /**When the input condition is met - end the command */
    @Override
    public boolean isFinished() {
        return m_until.get();
    }

    /**When the command ends (that is the condition is met) or get intterupted */
    @Override
    public void end(boolean interrupted) {
        m_constant.setValue(m_initValue);
    }
}