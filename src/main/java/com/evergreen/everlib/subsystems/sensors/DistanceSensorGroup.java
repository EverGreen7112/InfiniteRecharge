package com.evergreen.everlib.subsystems.sensors;

import java.util.ArrayList;
import java.util.List;

import com.evergreen.everlib.shuffleboard.loggables.LoggableData;
import com.evergreen.everlib.shuffleboard.loggables.LoggableDouble;
import com.evergreen.everlib.shuffleboard.loggables.LoggableObject;

/**
 * SensorGroup
 */
public class DistanceSensorGroup extends DistanceSensor implements LoggableObject {
    List<DistanceSensor> m_sesnsors;
    int sum;

    public DistanceSensorGroup(DistanceSensor... sensors) {
        m_sesnsors = List.of(sensors);
    }

    @Override
    public double _getDistance() {
        int sum = 0;


        for (DistanceSensor sensor : m_sesnsors) {

            if (sensor.killed()) {
                m_sesnsors.remove(sensor);
                continue;
            }

            sum += sensor.getDistance();
        }

        return sum / m_sesnsors.size();
    }

    @Override
    public String getName() {
        return getSubsystem() + " - Distance sensor";
    }

    @Override
    public List<LoggableData> getLoggableData() {
        List<LoggableData> loggables = new ArrayList<>();

        for (DistanceSensor sensor : m_sesnsors) { 
            loggables.add(new LoggableDouble(getSubsystem() + "/sensors/" + sensor.toString(),
            sensor::getDistance));
        }

        return loggables;
    }
}