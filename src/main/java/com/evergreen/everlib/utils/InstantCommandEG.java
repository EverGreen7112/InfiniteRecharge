package com.evergreen.everlib.utils;

import com.evergreen.everlib.CommandEG;

/**
 * InstantCommandEG
 */
public class InstantCommandEG extends CommandEG {

    Runnable m_toRun;

    public InstantCommandEG(String name, Runnable command) {
        super(name);
        m_toRun = command;
    }

    @Override
    public void initialize() {
        m_toRun.run();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}