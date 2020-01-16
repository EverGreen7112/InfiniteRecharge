/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.evergreen.everlib.subsystems.sensors;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AnalogInput;


/**
 * Add your docs here.
 */
public class LaserSensor extends DistanceSensor {
    
    private AnalogInput sensor;
    private Supplier<Double> m_slope, m_intercept;

    public LaserSensor(int port, double slope, double intercept) {
        sensor = new AnalogInput(port);
        m_slope = () -> slope;
        m_intercept = () -> intercept;
    }

    public LaserSensor(int port, Supplier<Double> slope, Supplier<Double> intercept)
    {
        sensor = new AnalogInput(port);
        m_slope = slope;
        m_intercept = intercept;
    }

    public void setConverterByPoints(double voltage1, double distance1, double voltage2, double distance2)
    {
        double slope = (distance2 - distance1) / (voltage2 - voltage1);
        m_slope = () -> slope;
        m_intercept = () -> distance2 - (voltage2 * slope);
    }

    public void setConverterByPoints(Point point1, Point point2) {
        setConverterByPoints(point1.m_voltage, point1.m_distance, point2.m_voltage, point2.m_distance);
    }

    public void setSlope(double value) {
        m_slope = () -> value;
    }

    public void setIntercept(double value) {
        m_intercept = () -> value;
    }
    
    class Point
    {
        double m_voltage;
        double m_distance;

        Point(double voltage, double distance) {
            m_voltage = voltage;
            m_distance = distance;
        }
    }

    @Override
    public double _getDistance() {
        return sensor.getVoltage() * m_slope.get() + m_intercept.get();
    }
}
