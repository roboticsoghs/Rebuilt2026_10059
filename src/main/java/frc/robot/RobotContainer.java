// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.autos.EightPieceAutoFromCenter;
import frc.robot.autos.Nothing;
import frc.robot.autos.OneMeterSquare;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.commands.Controls;

public class RobotContainer {
    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.07) // Add a 5% deadband to drive and 7% to rotation
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    public final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final IndexerSubsystem indexer = new IndexerSubsystem();
    public final FuelSubsystem fuel = new FuelSubsystem();
    public final Vision vision = new Vision();

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final SendableChooser<Integer> autoDelaySelector = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();
        configureAutos();
    }

    private void configureAutos() {
        autoChooser.setDefaultOption("NOTHING", new Nothing());
        autoChooser.addOption("8P-CENTER", new EightPieceAutoFromCenter(drivetrain, drive, brake, vision, fuel, indexer, MaxSpeed, MaxAngularRate));
        autoChooser.addOption("OneMeterSquare", new OneMeterSquare(drivetrain, drive, brake, MaxSpeed, MaxAngularRate));

        autoDelaySelector.setDefaultOption("No delay", 0);
        autoDelaySelector.addOption("1sec", 1);
        autoDelaySelector.addOption("2sec", 2);
        autoDelaySelector.addOption("3sec", 3);
        autoDelaySelector.addOption("4sec", 4);
        autoDelaySelector.addOption("5sec", 5);

        SmartDashboard.putData("AUTO SELECTOR", autoChooser);
        SmartDashboard.putData("AUTO DELAY", autoDelaySelector);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        Command joystickCommand = new Controls(drivetrain, drive, brake, vision, fuel, indexer, joystick, MaxSpeed, MaxAngularRate);

        Command indexWhenReady = Commands.sequence(
            Commands.runOnce(() -> indexer.startShooterFeed(), indexer),
            Commands.waitSeconds(1),
            Commands.runOnce(() -> indexer.stop(), indexer)
        );

        joystick.rightTrigger(0.1).whileTrue(
            Commands.sequence(
                Commands.runOnce(() -> fuel.runUp(fuel.calcSpeedByDistance(vision.getZ())), fuel, vision).
                    alongWith(Commands.runOnce(() -> indexer.startHopperIntake(), indexer)),
                Commands.waitSeconds(1.3),
                indexWhenReady.repeatedly()
            ).finallyDo(() -> {
                indexer.stop();
                fuel.stop();
            })
        );

        joystick.b().whileTrue(
            Commands.run(() -> vision.faceAprilTag(drivetrain, drive, brake, MaxAngularRate), vision)
        );

        joystick.y().onTrue(
            Commands.runOnce(() -> {
                indexer.stop();
                fuel.stop();
            }, indexer, fuel)
        );

        joystick.leftBumper().onTrue(
            Commands.runOnce(() -> {
                indexer.startGroundOuttake();
                fuel.startGroundOuttake();
            }, indexer, fuel)
        );

        joystick.leftTrigger(0.1).whileTrue(
            Commands.runOnce(() -> {
                indexer.startHopperIntake();
                fuel.startHopperIntake();
            }, indexer, fuel)
        );

        joystick.leftTrigger().whileFalse(
            Commands.runOnce(() -> {
                indexer.stop();
                fuel.stop();
            }, indexer, fuel)
        );

        Command defaultCommand = Commands.parallel(
            joystickCommand
        );
        drivetrain.setDefaultCommand(defaultCommand);

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.povUp().onTrue(
        //     Commands.runOnce(() -> autoAngleActive = !autoAngleActive, vision)
        // );
        // joystick.povDown().whileTrue(
        //     Commands.run(() -> vision.adjustDistance(drivetrain, drive, brake))
        // );


        // reset the field-centric heading
        joystick.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    }

    public Command getAutonomousCommand() {
        return Commands.sequence(
            Commands.waitSeconds(autoDelaySelector.getSelected()),
            autoChooser.getSelected()
        );
    }
}
