package com.evergreen.robot;

/**
 * Utillities
 */
public class Utillities {
    /**
     * The aimer postion, true if in the upper position,false if in the lower position
     */
    //TODO: find starting positin
     private static boolean m_aimPosition = false; 
     /**
      * 
      * @return The aimer postion, true if in the upper position,false if in the lower position
      */
     public static boolean getAimPosition() {
         return m_aimPosition;
     } 
}