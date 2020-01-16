package com.evergreen.everlib.subsystems.pistons.subsystems;

import java.util.List;

import com.evergreen.everlib.subsystems.SubsystemEG;
import com.evergreen.everlib.shuffleboard.loggables.LoggableData;
import com.evergreen.everlib.shuffleboard.loggables.LoggableString;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * A generic subsystem which uses pistons for movement.
 * It can iterates between Two {@link DoubleSolenoid.Value states}: 
 * {@link DoubleSolenoid.Value#kForward Forward} and {@link DoubleSolenoid.Value#kReverse Reverse,}
 * and {@link DoubleSolenoid.Value#kOff Off},    
 */
public class PistonSubsystem extends SubsystemEG {

    private DoubleSolenoid m_piston;

    public PistonSubsystem(DoubleSolenoid piston, String name)
    {
        super(name);
        m_piston = piston;
    }

    public void toggle()
    {
        if(isForward())
        {
            setReverse();
        }

        else
        {
            setForward();
        }
    }

    public void setReverse()
    {
        set(Value.kReverse);
    }

    public void setForward()
    {
        set(Value.kForward);
    }

    public void setOff()
    {
        set(Value.kOff);
    }

    public boolean isReversed()
    {
        return m_piston.get().compareTo(Value.kReverse) == 0;
    }

    public boolean isForward()
    {
        
        return m_piston.get().compareTo(Value.kForward) == 0;
    }

    public boolean isOff()
    {
        return m_piston.get().compareTo(Value.kOff) == 0;
    }

    public void set(Value state)
    {
        if(m_subsystemSwitch.get())
        {
            m_piston.set(state);
        }
    }

    @Override
    public List<LoggableData> getLoggableData() {
        return List.of(new LoggableData[]
        {
            new LoggableString(getName() + "/State", () -> m_piston.get().toString())
        });
    }
}
