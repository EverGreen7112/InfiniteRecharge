package com.evergreen.everlib.utils;

/**
 * Add your docs here.
 */
@FunctionalInterface
public interface Adjuster<T> {
    public T adjust(T value);
}
