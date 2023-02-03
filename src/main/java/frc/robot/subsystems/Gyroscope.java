package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

public class Gyroscope extends SubsystemBase { 

  private static Gyroscope instance;
  public static AHRS navX;
  public static double zeroHeading;
  public static double zeroAngle;

  /** Creates a new NavXGyro. */
  private Gyroscope() {
    navX = new AHRS(SPI.Port.kMXP);

    zeroHeading = getNavHeading();
    zeroAngle = getNavAngle();
    System.out.println("Setup ZeroAngle " + zeroAngle); 
  }
  
  // Public Methods
  public static Gyroscope getInstance() {
    if (instance == null) {
      instance = new Gyroscope();
    }
    return instance;
  }

  public double getNavHeading() {
		double heading = navX.getFusedHeading();
		return heading;
	}

	public double getNavAngle() {
		double angle = navX.getAngle();
		return angle;
	}

	public void zeroNavHeading() {
		//navX.zeroYaw();
    navX.reset();
    zeroHeading = getNavHeading();
    zeroAngle = getNavAngle();
  }

  public double getZeroHeading(){
    return zeroHeading;
  }

  public double getZeroAngle(){
    return zeroAngle;
  }

  public double getPitch() {
    return navX.getPitch();
  }

  public double getRoll() {
    return navX.getRoll();
  }

  public double getRate() {
    return navX.getRate();
  }

  public Rotation2d getNavXRotation2D(){
    return Rotation2d.fromDegrees(navX.getAngle());
  }

  public double getTemp() {
    return navX.getTempC();
  }

    /*
      Note that the math in the getHeading method is used to invert the direction of 
      the gyro for use by wpilib which treats gyros backwards.
      Gyros are normally clockwise positive. Wpilib wants 
      counter-clockwise positive.
  */
  public double getHeading() {
     return Math.IEEEremainder(-getNavAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }
}