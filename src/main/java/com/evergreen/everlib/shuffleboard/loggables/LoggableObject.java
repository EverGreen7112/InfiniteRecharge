package com.evergreen.everlib.shuffleboard.loggables;

import java.util.List;

/**
 * LoggableObject
 */
public interface LoggableObject {

    /**
     * @return The name of the loggable object (all its data should be located in 
     * folder with that name)
     */
    public String getName();

    /**
     * @return A list of all the data relevant to the object, to put into the shuffleboard
     * collectively.
     */
    public List<LoggableData> getLoggableData();
}