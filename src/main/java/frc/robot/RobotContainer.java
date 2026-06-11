// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
// import frc.robot.autos.EightPieceAutoFromCenter;
import frc.robot.autos.Nothing;
// import frc.robot.autos.SideAutoRelativeLeft;
// import frc.robot.autos.SideAutoRelativeRight;
import frc.robot.commands.Controls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.Vision;

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
    public final Vision vision = new Vision(drivetrain);

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final SendableChooser<Integer> autoDelaySelector = new SendableChooser<>();
    private final SendableChooser<Double>  shooterSpeed = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();
        configureAutos();
    }

    private void configureAutos() {
        autoChooser.setDefaultOption("NOTHING", new Nothing());
        // autoChooser.addOption("8P-CENTER", new EightPieceAutoFromCenter(drivetrain, drive, brake, vision, fuel, indexer, MaxSpeed, MaxAngularRate));
        // autoChooser.addOption("8P-LEFTREL", new SideAutoRelativeLeft(drivetrain, drive, brake, vision, fuel, indexer, MaxSpeed, MaxAngularRate));
        // autoChooser.addOption("8P-RIGHTREL", new SideAutoRelativeRight(drivetrain, drive, brake, vision, fuel, indexer, MaxSpeed, MaxAngularRate));
        // autoChooser.addOption("OneMeterSquare", new OneMeterSquare(drivetrain, drive, brake, MaxSpeed, MaxAngularRate));

        autoDelaySelector.setDefaultOption("No delay", 0);
        autoDelaySelector.addOption("1sec", 1);
        autoDelaySelector.addOption("2sec", 2);
        autoDelaySelector.addOption("3sec", 3);
        autoDelaySelector.addOption("4sec", 4);
        autoDelaySelector.addOption("5sec", 5);

        SmartDashboard.putData("AUTO SELECTOR", autoChooser);
        SmartDashboard.putData("AUTO DELAY", autoDelaySelector);

        shooterSpeed.addOption("SINGLE", 0.5);
        shooterSpeed.addOption("DOUBLE", 0.53);
        shooterSpeed.addOption("MULTI", 0.56);
        shooterSpeed.addOption("65%", 0.65);
        shooterSpeed.addOption("70%", 0.70);
        shooterSpeed.addOption("SAFE_MAX", 0.8);
        shooterSpeed.addOption("UNSAFE_MAX", 1.0);

        SmartDashboard.putData("SHOOTER SPEED", shooterSpeed);
    }

    private void configureBindings() {
        Command joystickCommand = new Controls(drivetrain, drive, brake, vision, fuel, indexer, joystick, MaxSpeed, MaxAngularRate);

        // joystick.rightTrigger(0.1).whileTrue(
        //     Commands.parallel(
        //         Commands.run(() -> fuel.runUp(fuel.calcSpeedByDistance(vision.getZ())), fuel),

        //         Commands.run(() -> {
        //             if (fuel.isAtSetpoint(80)) indexer.startShooterFeed();
        //             else indexer.startHopperIntake();
        //         }, indexer)
        //     ).finallyDo(() -> {
        //         indexer.stop();
        //         fuel.stop();
        //     })
        // );

        joystick.leftTrigger(0.1).whileTrue(
            drivetrain.applyRequest(() -> {
                double omega = vision.calculateYawError() * MaxAngularRate;
                double x = vision.calculateDistanceError(0.45) * MaxSpeed;
                double y = vision.calculateHorizontalError() * MaxSpeed;

                return drive
                    .withVelocityX(x)
                    .withVelocityY(y)
                    .withRotationalRate(omega);
            })
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

        joystick.rightBumper().onTrue(
            Commands.runOnce(() -> {
                indexer.startHopperIntake();
                fuel.startHopperIntake();
            }, indexer, fuel)
        );

        joystick.rightBumper().onFalse(
            Commands.runOnce(() -> {
                indexer.stop();
                fuel.stop();
            }, indexer, fuel)
        );

        joystick.rightTrigger(0.1).whileTrue(
            // shoot
            Commands.parallel(
                Commands.run(() -> fuel.runUp(shooterSpeed.getSelected().doubleValue()), fuel),
                Commands.run(() -> {
                    if (fuel.isAtSetpoint(100)) {
                        indexer.startShooterFeed();
                    }
                })
            ).finallyDo(() -> {
                indexer.stop();
                fuel.stop();
            })
        );

        joystick.rightTrigger().whileFalse(
            Commands.runOnce(() -> {
                indexer.stop();
                fuel.stop();
            }, indexer, fuel)
        );

        // reset the field-centric heading
        joystick.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        Command defaultCommand = Commands.parallel(
            joystickCommand
        );
        // Command defaultCommand = drivetrain.applyRequest(() -> {
        //         double vx = logScale(-joystick.getLeftY(), Constants.DRIVE_DEADBAND, Constants.kDrive);
        //         double vy = logScale(-joystick.getLeftX(), Constants.DRIVE_DEADBAND, Constants.kDrive);
        //         double omega = logScale(-joystick.getRightX(), Constants.ROT_DEADBAND, Constants.kRot);

        //         SmartDashboard.putNumber("VelocityX Setpoint", vx*MaxSpeed);
        //         SmartDashboard.putNumber("VelocityY Setpoint", vy*MaxSpeed);
        //         SmartDashboard.putNumber("Angular Setpoint", omega*MaxAngularRate);

        //         // if (joystick.b().getAsBoolean()) omega = vision.calculateOmegaError();

        //         return drive
        //             .withVelocityX(vx * MaxSpeed)
        //             .withVelocityY(vy * MaxSpeed)
        //             .withRotationalRate(omega * MaxAngularRate);
        //     });
        drivetrain.setDefaultCommand(defaultCommand);

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
    }

    private static double logScale(double in, double deadband, double k) {
        if (Math.abs(in) < deadband) return 0.0;

        double sign = Math.signum(in);
        double a = (Math.abs(in) - deadband) / (1.0 - deadband);
        double scaled = Math.log1p(k * a) / Math.log1p(k);
        return sign * scaled;
    }

    public Command getAutonomousCommand() {
        Command seedHeading = Commands.runOnce(() -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Red) {
                drivetrain.seedFieldCentric(Rotation2d.fromDegrees(180));
            } else {
                drivetrain.seedFieldCentric(Rotation2d.fromDegrees(0));
            }
        });

        return Commands.sequence(
            seedHeading,
            Commands.waitSeconds(autoDelaySelector.getSelected()),
            autoChooser.getSelected()
        );
    }
}
