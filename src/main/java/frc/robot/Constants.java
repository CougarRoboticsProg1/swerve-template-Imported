// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(21); // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(25.5); // FIXME Measure and set wheelbase

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 3;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(267.71484375 + 180); 

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 1;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(179.384765625);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 14;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 4;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 2; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(88.857421875);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 2;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 1;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 4;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(153.720703125);

    //Module Constants
    public static final double wheelDiameter = 0.10033;
    public static final double driveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final boolean driveInverted = true;
    public static final double steerReduction = (15.0 / 32.0) * (10.0 / 60.0);
    public static final boolean steerInverted = true;

    //Mk4 Module Configuration
    public static double nominalVoltage = 12.0;
    public static double driveCurrentLimit = 20.0;
    public static double steerCurrentLimit = 20.0;


  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = (MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0) ) ;

  public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );


    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    //Auto Constants
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 3;
        public static final double kPTranslationController = 2.8;
        public static final double kDTranslationController = 0.2;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 2,
                        MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 4);
}
