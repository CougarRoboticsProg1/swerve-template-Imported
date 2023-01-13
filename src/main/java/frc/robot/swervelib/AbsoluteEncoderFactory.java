package frc.robot.swervelib;

import frc.robot.swervelib.ctre.CanCoderAbsoluteConfiguration;

@FunctionalInterface
public interface AbsoluteEncoderFactory {
    AbsoluteEncoder create(CanCoderAbsoluteConfiguration configuration);
}
