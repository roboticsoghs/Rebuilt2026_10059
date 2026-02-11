package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.Constants;

public class Controls extends ParallelCommandGroup{

    // Is running variable
    public boolean autoAngleActive = false;

    public Controls(
        CommandSwerveDrivetrain drivetrain, 
        FieldCentric drive, 
        SwerveDriveBrake brake, 
        Vision vision,
        FuelSubsystem fuel, 
        IndexerSubsystem indexer, 
        XboxController joystick,
        double MaxSpeed,
        double MaxAngRate
    ) {
       addCommands(
            // ground intake
            Commands.run(() -> {
                indexer.startHopperIntake();
                fuel.startHopperIntake();
            }, indexer, fuel).onlyWhile(() -> joystick.getLeftTriggerAxis() >= 0.1),

            // Ground outtake
            Commands.run(() -> {
                indexer.startGroundOuttake();
                fuel.startGroundOuttake();
            }).onlyWhile(() -> joystick.getLeftBumperButton()),

            // spin up shooter
            Commands.run(() -> fuel.runUp()).onlyWhile(() -> joystick.getRightBumperButton()),

            // feed shooter
            Commands.run(() -> indexer.startShooterFeed()).onlyWhile(() -> joystick.getRightTriggerAxis() >= 0.1),

            // stop fuel system
            Commands.run(() -> {
                indexer.stop();
                fuel.stop();
            }).onlyWhile(() -> joystick.getYButton()),

            // Driving Controls (vroom vroom)
            // Commands.run(() -> {
            //     vision.faceAprilTag(drivetrain, drive, brake);
            // }, vision).onlyWhile(() -> autoAngleActive && vision.isChuteTag()),
            drivetrain.applyRequest(() -> {
                double vx = logScale(-joystick.getLeftY(), Constants.DRIVE_DEADBAND, Constants.kDrive);
                double vy = logScale(-joystick.getLeftX(), Constants.DRIVE_DEADBAND, Constants.kDrive);
                double omega = logScale(-joystick.getRightX(), Constants.ROT_DEADBAND, Constants.kRot);

                SmartDashboard.putNumber("VelocityX Setpoint", vx*MaxSpeed);
                SmartDashboard.putNumber("VelocityY Setpoint", vy*MaxSpeed);
                SmartDashboard.putNumber("Angular Setpoint", omega*MaxAngRate);

                return drive    
                    .withVelocityX(vx * MaxSpeed)
                    .withVelocityY(vy * MaxSpeed)
                    .withRotationalRate(omega * MaxAngRate);
            })
        );
    }

    // Log scale but funky
    private static double logScale(double in, double deadband, double k) {
        if (Math.abs(in) < deadband) return 0.0;

        double sign = Math.signum(in);
        double a = (Math.abs(in) - deadband) / (1.0 - deadband);
        double scaled = Math.log1p(k * a) / Math.log1p(k);
        return sign * scaled;
    }
}
