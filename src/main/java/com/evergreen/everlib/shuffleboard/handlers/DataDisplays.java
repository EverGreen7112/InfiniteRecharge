package com.evergreen.everlib.shuffleboard.handlers;

/**
 * DataDisplays
 */
public interface DataDisplays {
    
    public enum BoolDisplay
    {
        COLOR("Boolean Box"),
        TEXT("Text View"),
        BUTTON("Toggle Button"),
        SWITCH("Toggle Switch");
        
        String value;

        BoolDisplay(String value)
        {
            this.value = value;
        }
    }

    public enum NumberDisplay
    {
        GRAPH("Graph"),
        BAR("Number Bar"),
        SLIDER("Number Slider"),
        DIAL("Simple Dial"),
        TEXT("Text View"),
        VOLTAGE("Voltage View");


        String value;

        NumberDisplay(String value)
        {
            this.value = value;
        }
    }
}