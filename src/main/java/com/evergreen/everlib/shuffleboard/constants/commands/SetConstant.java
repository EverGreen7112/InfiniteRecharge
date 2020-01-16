package com.evergreen.everlib.shuffleboard.constants.commands;

import java.util.function.Supplier;

import com.evergreen.everlib.shuffleboard.constants.ConstantDouble;
import com.evergreen.everlib.utils.InstantCommandEG;

/**
 * A command to set a {@link ConstantDouble shuffleboard double constant} a certain value.
 * 
 * @author Atai Ambus
 */
public class SetConstant extends InstantCommandEG {

    /**
     * Constructs a new {@link SetConstant} command, which sets an input {@link ConstantDouble 
     * Shuffleboard double constant} an input value.
     * 
     * @param name - The name for this command (for switch and logging)
     * @param constant - The constant to set
     * @param value - The value to set it
     */
    public SetConstant(String name, ConstantDouble constant, double value) {
        super(name, () -> constant.setValue(value));
    }


    /**
     * Constructs a new {@link SetConstant} command, which sets an input {@link ConstantDouble 
     * Shuffleboard double constant} an input value.
     * 
     * @param name - The name for this command (for switch and logging)
     * @param constant - The constant to set
     * @param valueSupplier - A supplier of the value to set it to. (for example a 
     * {@link ConstantDouble})
     */
    public SetConstant(String name, ConstantDouble constant, Supplier<Double> valueSupplier) {
        super(name, () -> constant.setValue(valueSupplier.get()));
    }
}