/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.evergreen.everlib.oi.joysticks;

import com.evergreen.everlib.utils.Adjuster;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * A class for <a href="https://www.logitechg.com/en-roeu/products/gamepads/extreme-3d-pro-joystick.html">
 * Logitech's Extreme 3D Pro Joystick</a>, using our knowledge of the model to create easy 
 * access to its buttons (see {@link #getButton(X, Y, Z)})
 * <p><p>
 * Note that this class inherits {@link JoystickEG}, so it can be constructed with an
 * {@link Adjuster} which will automatically adjust its getRawAxis values, and can be easily
 * exponentiated with {@link #setExponential()} and {@link #setInverted()}. 
 * 
 * @author Atai Ambus
 */
public class ExtremeProJoystick extends JoystickEG {

    /**The port at which the normal button ports start, after the trigger and thumb buttons. */
    private int BUTTON_START = 3;

    /**
     * Constructs an {@link ExtremeProJoystick} object at given port.
     * 
     * @param port - The joystick's port, as tuned in the DriverStation.
     */
    public ExtremeProJoystick(String name, int port)
    {
        super(name, port);
    }

    /**
     * Constructs an {@link ExtremeProJoystick} object at given port,
     * its values adjusted by input {@link Adjuster}
     * 
     * @param port - The joystick port, as tuned in the DriverStation
     * @param adjuster - The function to adjust Joystick output.
     */
    public ExtremeProJoystick(String name, int port, Adjuster<Double> adjuster)
    {
        super(name, port, adjuster);
    }

    /**An {@link ExtremeProJoystick} button position on the x axis - left or right*/
    public enum X {
        LEFT(0),
        RIGHT(1);

        /**The port id in which */
        private int m_portsStart;

        private X(int portStart) {
            m_portsStart = portStart;
        }

        /**
         * @return the position within the current z-y cycle in which this X section starts. 
         */
        public int getStart() {
            return m_portsStart;
        }
    }
    
    /**A section of buttons on the joystick - either forward (farthest away form driver),
     * back (closest to driver), or middle (between right and left).
     * <p>
     * Each y section divides further into {@link X Left and Right buttons}.
    */
    public enum Y {
        FORWARD(0),
        MIDDLE(1),
        BACK(2);

        private int m_portsStart;

        private Y(int portStart) {
            m_portsStart = portStart;
        }

        /**
         * Calculates the position in this Z cycle this Y section starts. <p>
         * 
         * Since the top part of the joystick does not have a middle button, the Z section
         * is required as a parameter.<p>
         * If a middle button is requested in the top section, 
         * an exception will be thrown
         * @param section - The Z cycle (Top or bottom) the requested button is in.
         * @return  - The position in a z cycle this section (forward, middle or back) starts.
         */
        public int getStart(Z section) {

            if (section.equals(Z.TOP)) {
                if (this.equals(MIDDLE)) 
                    throw new IllegalArgumentException("There is no middle button at"
                    + " an ExtremeProJoystick's top section!");
                if (this.equals(BACK))
                    return m_portsStart - 1;
            }

            return m_portsStart;
        }
    }

    /**A section of buttons on the joystick - either the top, right on the joystick itself, 
     * or bottom, on the base of the joystick.
     * <p>
     * Each Z section divides further by closeness to the driver ({@link Y forward, back, or middle})
     */
    public enum Z {
        TOP(3),
        BOTTOM(7);

        private int m_portsStart;

        private Z(int portStart) {
            m_portsStart = portStart;
        }

        /**
         * @return - The port at which this Z cycle (top or bottom) starts.
         */
        public int getStart() {
            return m_portsStart;
        }


    }

    /**
     * Gets a {@link JoystickButton} from this joystick by input {@link X}, {@link Y} and {@link Z}
     * sections <p>
     * 
     * The buttons of each {@link ExtremeProJoystick} are divided thrice - once on the Z axis, 
     * into top (on top of the joystick and on its base), then in each top and bottom divided
     * into back (closest to driver) and forward (furthest from driver). In the bottoms ection 
     * it is divided more, to also a middle section (between forward and back). Each Y swction 
     * is divided once more - into Left and Right buttons (X axis.)
     * 
     * As such we require three -
     * 
     * @param x - The button's position on the X axis - left or right.
     * @param y - The button's section on the Y axis - forward, middle or back.
     * @param z - The button's section on the Z axis - top or bottom
     *  
     * @return - The {@link JoystickButton} at the matching position.
     */
    public Button getButton(X x, Y y, Z z) {
        return new JoystickButton(
            this, BUTTON_START + x.getStart() + y.getStart(z) + z.getStart());
    }

    /**
     * @return - this joystick's trigger button
     */
    public Button trigger() {
        return new JoystickButton(this, 1);
    }

    /**
     * @return - this joystick's thumb button.
    */
    public Button thumb() {
        return new JoystickButton(this, 2);
    }
}
