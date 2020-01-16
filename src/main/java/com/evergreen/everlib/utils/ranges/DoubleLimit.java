package com.evergreen.everlib.utils.ranges;

import java.util.function.Supplier;

/**
 * DoubleLimit
 */
public class DoubleLimit implements Range {

    private Supplier<Double> m_maxLimit, m_minLimit, m_tolerance;


    public DoubleLimit(Supplier<Double> maxLimit, Supplier<Double> minLimit) {
        m_maxLimit = maxLimit;
        m_minLimit = minLimit;
        m_tolerance = () -> 0.0;
    }

    public DoubleLimit(Supplier<Double> maxLimit, Supplier<Double> minLimit, Supplier<Double> tolerance) {
        m_maxLimit = maxLimit;
        m_minLimit = minLimit;
        m_tolerance = tolerance;
    }

    @Override
    public boolean inRange(double value) {
        return value > m_minLimit.get() - m_tolerance.get()
            && value < m_maxLimit.get() + m_tolerance.get();
    }
}