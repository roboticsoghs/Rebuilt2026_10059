package frc.robot.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.Vision;

public class EightPieceAutoFromCenter extends SequentialCommandGroup {
    public EightPieceAutoFromCenter(
        CommandSwerveDrivetrain drivetrain,
        SwerveRequest.FieldCentric drive,
        SwerveRequest.SwerveDriveBrake brake,
        Vision vision,
        FuelSubsystem fuel,
        IndexerSubsystem indexer,
        double MaxSpeed,
        double MaxAngRate
    ) {
        addCommands(
            drivetrain.applyRequest(() -> drive.withVelocityX(0.5 * MaxSpeed)).until(() -> vision.isChuteTag()),
            drivetrain.applyRequest(() -> brake),
            Commands.run(() -> vision.adjustDistance(drivetrain, drive, brake))
                .andThen(() -> vision.faceAprilTag(drivetrain, drive, brake)),
            Commands.run(() -> fuel.runUp()),
            Commands.waitSeconds(0.5),
            Commands.run(() -> indexer.startShooterFeed())
        );
    }
}