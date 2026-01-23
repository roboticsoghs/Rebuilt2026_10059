// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class RobotContainer {
    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.07) // Add a 5% deadband to drive and 7% to rotation
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    public final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public final CommandXboxController joystick = new CommandXboxController(0);

    // public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private final IndexerSubsystem indexer = new IndexerSubsystem();

    private final FuelSubsystem fuel = new FuelSubsystem();

    // joystick command configs
    // TODO: move joystick command to seperate command file
    private static final double DRIVE_DEADBAND = 0.03;
    private static final double ROT_DEADBAND = 0.03;
    private static final double kDrive = 5;
    private static final double kRot = 7;
    public static final Subsystem Drivetrain = null;

    public RobotContainer() {
        configureBindings();
        configureAutos();
    }

    private void configureAutos() {
        // autoChooser.setDefaultOption("NOTHING", Commands.none());
        // autoChooser.addOption("OneMeterForward", new DriveToPoseCommand(drivetrain, drive, brake, 1, 0, 0, MaxSpeed, MaxAngularRate));
        // autoChooser.addOption("OneMeterSquare", Commands.sequence(
        //     new DriveToPoseCommand(drivetrain, drive, brake, -2, 0, 0, MaxSpeed, MaxAngularRate), // backward 0.5m
        //     new DriveToPoseCommand(drivetrain, drive, brake, 0, -2, 0, MaxSpeed, MaxAngularRate), // move right 0.5m
        //     new DriveToPoseCommand(drivetrain, drive, brake, 2, 0, 0, MaxSpeed, MaxAngularRate), // move forward 0.5m
        //     new DriveToPoseCommand(drivetrain, drive, brake, 0, 2, 0, MaxSpeed, MaxAngularRate) // move left 0.5m
        // ));
        // autoChooser.addOption("Surprise", Commands.sequence(
        //     new DriveToPoseCommand(drivetrain, drive, brake, 0, 1, 0, MaxSpeed, MaxAngularRate), 
        //     new DriveToPoseCommand(drivetrain, drive, brake, -2, 0, 0, MaxSpeed, MaxAngularRate), 
        //     new DriveToPoseCommand(drivetrain, drive, brake, 0, -1, 0, MaxSpeed, MaxAngularRate), 
        //     new DriveToPoseCommand(drivetrain, drive, brake, 2, 0, 0, MaxSpeed, MaxAngularRate) 
        // ));

        // SmartDashboard.putData("AUTO SELECTOR", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() -> {
        //         double vx = logScale(-joystick.getLeftY(), DRIVE_DEADBAND, kDrive);
        //         double vy = logScale(-joystick.getLeftX(), DRIVE_DEADBAND, kDrive);
        //         double o = logScale(-joystick.getRightX(), ROT_DEADBAND, kRot);

        //         SmartDashboard.putNumber("VelocityX Setpoint", vx * MaxSpeed);
        //         SmartDashboard.putNumber("VelocityY Setpoint", vy * MaxSpeed);
        //         SmartDashboard.putNumber("Omega Setpoint", o * MaxAngularRate);

        //         return drive
        //             .withVelocityX(vx * MaxSpeed)
        //             .withVelocityY(vy * MaxSpeed)
        //             .withRotationalRate(o * MaxAngularRate);
        //     })
        // );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        // final var idle = new SwerveRequest.Idle();
        // RobotModeTriggers.disabled().whileTrue(
        //     drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        // );


        // joystick.a().whileTrue(
        //     drivetrain.applyRequest(() -> {
        //         Pose2d start = drivetrain.getStartingPose();
        //         if (start == null) return drive.withVelocityX(0).withVelocityY(0);

        //         double[] vel = drivetrain.calculateDriveToPose(start);
        //         if (vel == null) return brake;

        //         // smooth out speeds using logarithmic scaling like joysticks
        //         double vx = logScale(vel[0], DRIVE_DEADBAND, kDrive);
        //         double vy = logScale(vel[1], DRIVE_DEADBAND, kDrive);
        //         double o = logScale(vel[2], ROT_DEADBAND, kRot);

        //         SmartDashboard.putNumber("x", vx);
        //         SmartDashboard.putNumber("y", vy);
        //         SmartDashboard.putNumber("r", o);

        //         // travel to home at 40% speed, rotate at 60% rate
        //         return drive.withVelocityX(vx * (MaxSpeed * 0.4))
        //                     .withVelocityY(vy * (MaxSpeed * 0.4))
        //                     .withRotationalRate(o * (MaxAngularRate * 0.6));
        //     })
        // );

        joystick.b().whileTrue(
            Commands.run(() -> {
                indexer.spinUp(0.7);
            }, indexer)
        );

        joystick.x().whileTrue(
            Commands.run(() -> {
                indexer.spinUp(-0.7);
            }, indexer)
        );

        joystick.rightTrigger(0.1).whileTrue(
            Commands.run(() -> {
                fuel.spinUp(1.0);
            }, fuel)
        );

        joystick.leftTrigger(0.1).whileTrue(
            Commands.run(() -> {
                fuel.spinUp(0.75);
            }, fuel)
        );

        joystick.y().onTrue(
            Commands.runOnce(() -> {
                indexer.stop();
                fuel.stop();
            })
        );

        // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private static double logScale(double in, double deadband, double k) {
        if (Math.abs(in) < deadband) return 0.0;

        double sign = Math.signum(in);
        double a = (Math.abs(in) - deadband) / (1.0 - deadband);
        double scaled = Math.log1p(k * a) / Math.log1p(k);
        return sign * scaled;
    }
}
