package frc.robot.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class OneMeterSquare extends SequentialCommandGroup {
    public OneMeterSquare(
        CommandSwerveDrivetrain drivetrain,
        SwerveRequest.FieldCentric drive,
        SwerveRequest.SwerveDriveBrake brake,
        double MaxSpeed,
        double MaxAngRate
    ) {
        addCommands(
            new DriveToPoseCommand(drivetrain, drive, brake, -2, 0, 0, MaxSpeed, MaxAngRate), 
            new DriveToPoseCommand(drivetrain, drive, brake, 0, -2, 0, MaxSpeed, MaxAngRate), 
            new DriveToPoseCommand(drivetrain, drive, brake, 2, 0, 0, MaxSpeed, MaxAngRate),
            new DriveToPoseCommand(drivetrain, drive, brake, 0, 2, 0, MaxSpeed, MaxAngRate)
        );
    }
}