package com.evergreen.everlib;

import java.util.List;

import com.evergreen.everlib.shuffleboard.constants.ConstantBoolean;
import com.evergreen.everlib.shuffleboard.loggables.LoggableData;
import com.evergreen.everlib.shuffleboard.loggables.LoggableInt;
import com.evergreen.everlib.shuffleboard.loggables.LoggableObject;
import com.evergreen.everlib.subsystems.SubsystemEG;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * The basic command for the Eververgreen Framework.
 * 
 * It contains a switch and will only start if it is on.
 * 
 * @author Atai Ambus
 */
public abstract class CommandEG extends CommandBase implements LoggableObject {
    private ConstantBoolean m_commandSwitch;

    private int m_ranCounter = 0;
    
    /**
     * Constructs a new {@link CommandEG} with input name, and without logging it in the shuffleboard
     * @param name - The subsystem's name, corresponding to is {@link #getName()} method
     * as well as its {@link ConstantBoolean Shuffleboard Switch}.
     */
    public CommandEG(String name) {
        setName(name);
        m_commandSwitch = new ConstantBoolean(name + "/switch");
    }

    
    /**
     * Constructs a new {@link CommandEG} with input name.
     * 
     * @param name - The subsystem's name, corresponding to is {@link #getName()} method
     * as well as its {@link ConstantBoolean Shuffleboard Switch}.
     * 
     * @param subsystems - Any subsystems the command requires.
     */
    public CommandEG(String name, SubsystemEG... subsystems) {
        this(name);
        
        for (SubsystemEG subsystem : subsystems) {
            addRequirements((Subsystem)subsystem);
        }

        m_commandSwitch = new ConstantBoolean(subsystems[0].getName() + "/command switches/" + name);
    }

    /**Schedules this command, defaulting to interruptible, as long both this an*/
    @Override
    public void schedule() {
        m_ranCounter++;

        if (canStart())
            super.schedule();
    }


    /**
     * Disables the command and prevents it from starting.
     */
    public void disable() {
        m_commandSwitch.disable();
    }

    /**
     * Enables the command, allowing it to start.
     */
    public void enable() {
        m_commandSwitch.enable();
    }

    /**
     * Toggles the command - if it is enabled disable it, and vice versa.
     */
    public void toggle() {
        m_commandSwitch.toggle();
    }

    /** 
     * @return the command's switch.
     */
    public ConstantBoolean getSwitch() {
        return m_commandSwitch;
    }

    @Override
    public boolean isFinished() {
        return !m_commandSwitch.get();
    }


    private boolean canStart() {
        
        for (Subsystem subsystem : getRequirements()) {
            if (subsystem instanceof SubsystemEG) {
                SubsystemEG sub = (SubsystemEG)subsystem;
                if (!sub.getSwitchState())
                    return false;
            }
        }

        return m_commandSwitch.get();
    }

    @Override
    public List<LoggableData> getLoggableData() {
        return List.of(new LoggableData[] {
            new LoggableInt(getName() + "/Ran Counter", () -> m_ranCounter)
        });
    }

    @Override
    public void initSendable(SendableBuilder builder) { }
}