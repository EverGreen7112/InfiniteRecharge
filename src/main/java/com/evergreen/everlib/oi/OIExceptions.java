/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.evergreen.everlib.oi;

/**
 * The Class containing all custom OI exceptions used in Everlib. 
 */
public class OIExceptions {

    public static class AxisDoesNotExistException extends RuntimeException
    {
        private static final String defaultMessage = "The requested axis does not exist in input joystick!";

        private static final long serialVersionUID = 1L;

        public AxisDoesNotExistException() {
            super(defaultMessage);
        }

        public AxisDoesNotExistException(Throwable err) {
            super(defaultMessage, err);
        }

        public AxisDoesNotExistException(String message) {
            super(message);
        }
        
        public AxisDoesNotExistException(String message, Throwable err) {
            super(message, err);
        }
    }
}
