package com.evergreen.everlib.shuffleboard.loggables;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.evergreen.everlib.shuffleboard.handlers.Explorer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * DashboardStreams
 */
public class DashboardStreams {

    private static final List<LoggableData> m_loggables = new ArrayList<>();
    private static Explorer m_explorer = new Explorer();

    public static void log(LoggableData loggable) {
        String name = loggable.getKey();
        String key = m_explorer.pwd() + name;

        if (!SmartDashboard.containsKey(key)) {
            loggable.setKey(key);
            loggable.addToDashboard();
            m_loggables.add(loggable);
        }

        else {
            System.out.println(String.format("Tried to add \"%s\" %s value to the shuffleboard under "
                    + " %s, but couldn't becuase there was already a value under that "
                    + "key.", name, loggable.getType(), pwd()));
        }
    }

    public static void addString(String key, Supplier<String> valueSupplier) {
        log(new LoggableString(key, valueSupplier));
    }

    public static void addInt(String key, Supplier<Integer> valueSupplier) {
        log(new LoggableInt(key, valueSupplier));
    }

    public static void addDouble(String key, Supplier<Double> valueSupplier) {

        log(new LoggableDouble(key, valueSupplier));
    }

    public static void addBoolean(String key, Supplier<Boolean> valueSupplier) {

        log(new LoggableBoolean(key, valueSupplier));
    }

    public static void addLoggable(LoggableObject loggable) {

        DashboardStreams.cd(loggable.getName());

        for (LoggableData loggableData : loggable.getLoggableData()) 
        {
            log(loggableData);
        }

        DashboardStreams.cd("..");
    }


    public static void update() {
        for (LoggableData loggableData: m_loggables) {
            loggableData.addToDashboard();
        }
    }

    public static String pwd() {
        return m_explorer.pwd();
    }
    
    public static Explorer cd(String path) {
        return m_explorer.cd(path);
    }



}