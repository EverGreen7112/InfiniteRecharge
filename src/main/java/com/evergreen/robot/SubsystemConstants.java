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
    public static class ShooterConstant{
       public static final ConstantDouble 
       /**
        * the speed that used to raise the aimer
        */
        //TODO: add correct speed
        aimingUpSpeed = new ConstantDouble("aimingUpSpeed", 0.5),
        /**
        * the speed that used to move down the aimer
        */
        //TODO: add correct speed
        aimingDownSpeed = new ConstantDouble("aimingDownSpeed", -0.3),
        /**
        * the time(secends) that takes to raise the aimer
        */
        //TODO: add correct time
        aimingUpTime = new ConstantDouble("aimingUpTime", 3),
        /**
        * the time(secends) that takes to move down the aimer
        */
        //TODO: add correct time
        aimingDownTime = new ConstantDouble("aimingDownTime", 2.5);
        
       

        
    }
    
}