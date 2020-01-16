package com.evergreen.everlib.utils.ranges;

import java.util.function.Supplier;

/**
 * MaxLimit
 */
public class MaxLimit implements Range {

    private Supplier<Double> m_maxLimit, m_tolerance;

    public MaxLimit(double maxLimit) {
        m_maxLimit = () -> maxLimit;
        m_tolerance = () -> 0.0;
    }

    public MaxLimit(Supplier<Double> limit) {
        m_maxLimit = limit;
        m_tolerance = () -> 0.0;
    }

    public MaxLimit(Supplier<Double> limit, Supplier<Double> tolerance) {
        m_maxLimit = limit;
        m_tolerance = tolerance;
    }

    public MaxLimit(Supplier<Double> limit, double tolerance) {
        m_maxLimit = limit;
        m_tolerance = () -> tolerance;
    }

    @Override
    public boolean inRange(double value) {
        return value < m_maxLimit.get() + m_tolerance.get();
    }
}