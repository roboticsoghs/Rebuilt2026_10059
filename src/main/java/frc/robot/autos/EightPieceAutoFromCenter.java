package frc.robot.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
            // Commands.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.fromDegrees(180))),
            drivetrain.applyRequest(() -> drive.withVelocityX(-0.5 * MaxSpeed))
                .until(() -> vision.isAnyAllianceHubFront())
                .withTimeout(0.4),
            drivetrain.applyRequest(() -> brake).withTimeout(0.1),
            Commands.run(() -> vision.faceAprilTag(drivetrain, drive, brake, MaxAngRate), vision, drivetrain)
                .until(() -> vision.isFacingAprilTag())
                .finallyDo(() -> drivetrain.setControl(brake))
                .withTimeout(0.75),
            Commands.parallel(
                Commands.runOnce(() -> fuel.runUp(0.72), vision, fuel),
                Commands.runOnce(() -> indexer.startHopperIntake(), indexer),
                Commands.waitSeconds(1.5)
            ),
            Commands.runOnce(() -> indexer.startShooterFeed(), indexer).repeatedly().withTimeout(17),
            Commands.parallel(
                Commands.runOnce(() -> {
                    fuel.stop();
                    indexer.stop();
                }, fuel, indexer)
            )
        );
    }
}