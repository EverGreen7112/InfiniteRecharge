/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.evergreen.everlib.utils.ranges;

import java.util.function.Supplier;

public class MinLimit implements Range {
    
    Supplier<Double> m_minDistance;
    Supplier<Double> m_tolerance;

    public MinLimit(double minDistance) {
        m_minDistance = () -> minDistance;
    }

    public MinLimit(Supplier<Double> minDistance) {
        m_minDistance = minDistance;
    }

    public MinLimit(Supplier<Double> minDistance, Supplier<Double> tolerance) {
        m_minDistance = minDistance;
        m_tolerance = tolerance;
    }

    public MinLimit(double minDistance, double tolerance) {
        m_minDistance = () -> minDistance;
        m_tolerance = () -> tolerance;
    }
    
    @Override
    public boolean inRange(double distance) {
        return distance > m_minDistance.get() - m_tolerance.get();
    }
}