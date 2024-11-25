package org.firstinspires.ftc.teamcode.math_utils;

import org.firstinspires.ftc.teamcode.constants.OctoConstants;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class EncoderFallingEdge implements OctoConstants
{
    IntSupplier position;
    IntSupplier velocity;

    int numFullRotations;
    double prevPosition;

    public EncoderFallingEdge( IntSupplier position, IntSupplier velocity ) {
        this.position = position;
        this.velocity = velocity;
        prevPosition = 0.0;
        numFullRotations = 0;
    }

    public int calculatePosition() {
        if(prevPosition > 750 && position.getAsInt() < 200)
            numFullRotations++;

        if(prevPosition < 200 && position.getAsInt() > 750)
            numFullRotations--;

        prevPosition = position.getAsInt();

        return 1024 * numFullRotations + position.getAsInt();
    }

    public int getNumFullRotations() {
        return numFullRotations;
    }

    public void reset()
    {
        numFullRotations = 0;
    }
}
