package frc.robot.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.Vision;

public class SideAutoRelativeRight extends SequentialCommandGroup {
    public SideAutoRelativeRight(
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
            drivetrain.applyRequest(() -> drive.withRotationalRate(0.3 * MaxAngRate))
                .until(() -> vision.isAnyAllianceHubAnySide())
                .withTimeout(2),
            drivetrain.applyRequest(() -> brake).withTimeout(0.1),
            drivetrain.applyRequest(() -> drive.withRotationalRate(0)).withTimeout(0.1),
            Commands.run(() -> vision.faceAprilTag(drivetrain, drive, brake, MaxAngRate), vision, drivetrain)
                .until(() -> vision.isFacingAprilTag(-225.0))
                .onlyIf(() -> vision.isAprilTag())
                .finallyDo(() -> drivetrain.setControl(brake))
                .withTimeout(0.5),
            Commands.parallel(
                Commands.runOnce(() -> fuel.runUp(fuel.calcSpeedByDistance(vision.getZ()) - 0.05), vision, fuel),
                Commands.runOnce(() -> indexer.startHopperIntake(), indexer),
                Commands.waitSeconds(1.5)
            ),
            Commands.runOnce(() -> indexer.startShooterFeed(), indexer).repeatedly()
        );
    }
}