package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class TestPIDAuto extends CommandBase {

    private final SwerveSubsystem drivetrain;
    private final ProfiledPIDController controller;
    private double controllerCalculate;
    private ShuffleboardTab tab = Shuffleboard.getTab("Drive Tank PID");

    private NetworkTableEntry poseOfRobot, kP, kI, kD, setPoint;

    public TestPIDAuto(SwerveSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        kP = tab.add("kP", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

        kI = tab.add("kI", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

        kD = tab.add("kD", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        
        setPoint = tab.add("Setpoint", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        
        poseOfRobot = tab.add("Pose", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        
        controller = new ProfiledPIDController(0, 0, 0, Constants.kThetaControllerConstraints);
    }

    @Override
    public void execute() {
        controller.setP(kP.getDouble(0));
        controller.setI(kI.getDouble(0));
        controller.setD(kD.getDouble(0));

        controllerCalculate = controller.calculate(drivetrain.getGyroscopeRotation().getDegrees(), setPoint.getDouble(0));
        System.out.println(controllerCalculate);

        drivetrain.drive(
            new ChassisSpeeds(0, 0, controllerCalculate));
    }
}
