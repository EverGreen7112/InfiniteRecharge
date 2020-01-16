package com.evergreen.everlib.subsystems.sensors;

import com.evergreen.everlib.utils.ranges.Range;

import edu.wpi.first.wpilibj.Encoder;

/**
 * A wrapper class for {@link Encoder} which extends {@link DistanceSensor},
 * and as such allowes for easier calibration and error-proofing.
 * 
 * @author Atai Ambus
 */
public class EncoderEG extends DistanceSensor {

    /**
     * The wrapped encoder object.
     */
    private Encoder m_encoder;


    /**
     * Constructs an {@link EncoderEG} object according to input sources, encoder offset and 
     * limits.
     *      
     * @param encoder - The encoder to wrap.
     * @param absoluteLimit - The absolute limits that it should  be able to mesure. for example, an
     * elevator's height.
     * @param offset - The offset from the point of mesurement. For example, its height on an elevator.
     * @param distancePerPulse - the distance the subsystem passes with each encoder tick.
     */
    public EncoderEG(Encoder encoder, Range absoluteLimit, double offset, double distancePerPulse) {
        this(encoder, absoluteLimit, offset);
        m_encoder.setDistancePerPulse(distancePerPulse);
    }

    /**
     * Constructs an {@link EncoderEG} object according to input sources, encoder offset and 
     * limits.
     * @param encoder - The encoder to wrap.
     * @param absoluteLimit - The absolute limits that it should  be able to mesure. for example, an
     * elevator's height.
     * @param offset - The offset from the point of mesurement. For example, its height on an elevator.
     */
    //All other constructors call this one, fillinng the gaps with default values.
    public EncoderEG(Encoder encoder, Range absoluteLimit, double offset) {
        super(absoluteLimit, offset);
        m_encoder = encoder;
    }
    


    /**
     * Constructs an {@link EncoderEG} object according to input sources, encoder offset and 
     * limits.
     * @param portA - The first source of the encoder.
     * @param portB - The second source fo the encoder.
     * @param absoluteLimit - The absolute limits that it should  be able to mesure. for example, an
     * elevator's height.
     * @param offset - The offset from the point of mesurement. For example, its height on an elevator.
     */
    //All other constructors call this one, fillinng the gaps with default values.
    public EncoderEG(int portA, int portB, Range absoluteLimit, double offset) {
        this(new Encoder(portA, portB), absoluteLimit, offset);
    }


    /**
     * Constructs an {@link EncoderEG} object according to input sources, encoder offset and 
     * limits.
     * @param portA - The first source of the encoder.
     * @param portB - The second source fo the encoder.
     * @param absoluteLimit - The absolute limits that it should  be able to mesure. for example, an
     * elevator's height.
     * @param offset - The offset from the point of mesurement. For example, its height on an elevator.
     * @param distancePerPulse - the distance the subsystem passes with each encoder tick.
     */
    public EncoderEG(int portA, int portB, Range absoluteLimit, double offset, double distancePerPulse) {
        this(new Encoder(portA, portB), absoluteLimit, offset, distancePerPulse);
    }

    
    /**
     * Constructs an {@link EncoderEG} object according to input sources.
     * limits.
     * @param portA - The first source of the encoder.
     * @param portB - The second source fo the encoder.
     */
    public EncoderEG(int portA, int portB) {  
        this(portA, portB, (v) -> true, 0);
    }


    /**
     * Constructs an {@link EncoderEG} object according to input sources and encoder offset.
     * @param portA - The first source of the encoder.
     * @param portB - The second source fo the encoder.
     * @param offset - The offset from the point of mesurement. For example, its height on ann elevator.
     */
    public EncoderEG(int portA, int portB, double offset) {
        this(portA, portB, (v) -> true, offset);        
    }


    /**
     * Constructs an {@link EncoderEG} object according to input sources and limits.
     * @param portA - The first source of the encoder.
     * @param portB - The second source fo the encoder.
     * @param absoluteLimit - The absolute limits that it should  be able to mesure. for example, an
     * elevator's height.
     */
    public EncoderEG(int portA, int portB, Range absoluteLimit) {
        this(portA, portB, absoluteLimit, 0);
    }

    /**
     * Sets the distance per pulse according to an input value. 
     * @param value - the value to set.
     */
    public void setDistancePerPulse(double value) {
        m_encoder.setDistancePerPulse(value);
    }

    /** 
     * Sets the distance per pulse according to the encoder's ticks per mototr rotation and
     * the diameter of wheels.
     * @param ticksPerRevolution - the amount of ticks per motor revolutoion.
     * @param diameter - The amount of ticks per
    */
    public void setDistancePerPulse(double ticksPerRevolution, double diameter) {
        m_encoder.setDistancePerPulse(1/ticksPerRevolution * diameter * Math.PI);
    }

    /**Resets the encoder's distance to 0. */
    public void reset() {
        m_encoder.reset();
    }

    /**
     * Since {@link EncoderEG} is a wrapped class, and the encoder is usually used exclusively
     * for relatively simple distance mesurement, most of its methods were not implemented directly.
     * This method return the wrapped {@link Encoder} object for the programmer to use.
     * 
     * @return the wrapped encoder object.
     */
    public Encoder getEncoder() {
        return m_encoder;
    }

    @Override
    protected double _getDistance() {
        return m_encoder.getDistance();
    }
}