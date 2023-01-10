package frc.robot.swervelib;

import com.ctre.phoenix.sensors.CANCoder;

public class AbsoluteEncoder {
    private final CANCoder encoder;

    public AbsoluteEncoder(CANCoder encoder) {
            this.encoder = encoder;
    }

    /**
     * Gets the current angle reading of the encoder in radians.
     *
     * @return The current angle in radians. Range: [0, 2pi)
     */
    public double getAbsoluteAngle() {
        double angle = Math.toRadians(encoder.getAbsolutePosition());
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }
}
