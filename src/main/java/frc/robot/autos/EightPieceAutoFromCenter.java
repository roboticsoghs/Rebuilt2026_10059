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
        Command indexWhenReady = Commands.sequence(
            Commands.runOnce(() -> indexer.startShooterFeed(), indexer),
            Commands.waitSeconds(1.25),
            Commands.runOnce(() -> indexer.stop(), indexer)
        );

        addCommands(
            drivetrain.applyRequest(() -> drive.withVelocityX(0.5 * MaxSpeed))
                .until(() -> vision.isChuteTag())
                .withTimeout(2),
            drivetrain.applyRequest(() -> brake).withTimeout(0.1),
            Commands.run(() -> vision.faceAprilTag(drivetrain, drive, brake, MaxAngRate), vision, drivetrain)
                .until(() -> vision.isFacingAprilTag())
                .finallyDo(() -> drivetrain.setControl(brake)),
            Commands.parallel(
                Commands.runOnce(() -> fuel.runUp(fuel.calcSpeedByDistance(vision.getZ())), vision, fuel),
                Commands.runOnce(() -> indexer.startHopperIntake(), indexer),
                Commands.waitSeconds(1.5)
            ),
            indexWhenReady.repeatedly().withTimeout(17),
            Commands.parallel(
                Commands.runOnce(() -> {
                    fuel.stop();
                    indexer.stop();
                }, fuel, indexer)
            )
        );
    }
}