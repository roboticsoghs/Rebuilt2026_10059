package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveDistance extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final double speedMetersPerSecond;
    private final double distance;

    private final SwerveRequest.FieldCentric drive;

    private double startX;
    private double MaxSpeed;

    public DriveDistance(
        CommandSwerveDrivetrain drivetrain,
        double speedMetersPerSecond,
        double distance,
        double MaxSpeed,
        double MaxAngularRate
    ) {
        this.drivetrain = drivetrain;
        this.speedMetersPerSecond = speedMetersPerSecond;
        this.MaxSpeed = MaxSpeed;
        this.distance = distance;
        
        this.drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d pose = drivetrain.getPose();
        startX = pose.getX();
        drivetrain.seedFieldCentric();
    }

    @Override
    public void execute() {
        drivetrain.setControl(
            drive.withVelocityX(speedMetersPerSecond * MaxSpeed)
            .withVelocityY(0)
            .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        double currentX = drivetrain.getPose().getX();
        return (currentX - startX) >= distance;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }
}