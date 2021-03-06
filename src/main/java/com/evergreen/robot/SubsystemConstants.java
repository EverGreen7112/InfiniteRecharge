package com.evergreen.robot;

import com.evergreen.everlib.shuffleboard.constants.ConstantDouble;
import com.evergreen.everlib.shuffleboard.constants.DashboardConstants;

/**
 * SubsystemConstants
 */
public interface SubsystemConstants {

    /**
     * SubsystemAConstant
     */
    public static class ChassisConstants {

        static
        {
            DashboardConstants.getInstance().startConstantsOf("SubsystemA");
        }

        public static final ConstantDouble
                defaultSpeed = new ConstantDouble("Default Speed", 0.7),
                defenseSpeed = new ConstantDouble("Defense Speed", 0.4),
                emergencySpeed = new ConstantDouble("Emergency Speed", 0.85);


        // public static final double
        // ...

        // public static final ConstantDouble
        // ...
        
    }
    
    /**
     * SubsystemBConstant
     */
    public static class SubsystemBConstants {
        
        static
        {
            DashboardConstants.getInstance().startConstantsOf("SubsystemB");
        }

        // public static final ConstantDouble
        // ...

    }


    /**
     * SubsystemCConstants
     */
    public static class SubsystemCConstants {

        static
        {
            DashboardConstants.getInstance().startConstantsOf("SubsystemC");
        }

        // public static final ConstantDouble
        // ...
        
    }
}