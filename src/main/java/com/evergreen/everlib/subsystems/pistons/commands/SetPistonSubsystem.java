package com.evergreen.everlib.subsystems.pistons.commands;

import java.util.List;

import com.evergreen.everlib.CommandEG;
import com.evergreen.everlib.subsystems.pistons.subsystems.PistonSubsystem;
import com.evergreen.everlib.shuffleboard.loggables.LoggableData;
import com.evergreen.everlib.shuffleboard.loggables.LoggableString;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class SetPistonSubsystem extends CommandEG {
  /**Supplier for the state to be set.
   * When this command executes, the */
  private Value m_state;
  private PistonSubsystem m_pistons;
  
  public SetPistonSubsystem(String name, PistonSubsystem pistons, Value state) {
    super(name, pistons);
    m_state = state;
    m_pistons = pistons;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
      m_pistons.set(m_state);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished()  {
    return true;
  }

  @Override
  public List<LoggableData> getLoggableData() {

    List<LoggableData> loggables = super.getLoggableData();
    
    loggables.add(new LoggableString(getName() + " - subsystem", m_pistons::getName));
    loggables.add(new LoggableString(getName() + " - value to set", () -> m_state.toString()));

    return loggables;
  }
}