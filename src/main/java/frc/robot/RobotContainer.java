// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    private boolean autoAngleActive = false;

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
        autoChooser.setDefaultOption("NOTHING", new Nothing());
        autoChooser.addOption("8P-CENTER", new EightPieceAutoFromCenter(drivetrain, drive, brake, vision, fuel, indexer, MaxSpeed, MaxAngularRate));
        autoChooser.addOption("OneMeterSquare", new OneMeterSquare(drivetrain, drive, brake, MaxSpeed, MaxAngularRate));

        SmartDashboard.putData("AUTO SELECTOR", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        
        Command defaultCommand = Commands.parallel(
            Commands.run(() -> {
                vision.faceAprilTag(drivetrain, drive, brake);
            }, vision).onlyWhile(() -> autoAngleActive && vision.isChuteTag()),
            drivetrain.applyRequest(() -> {
                double vx = logScale(-joystick.getLeftY(), DRIVE_DEADBAND, kDrive);
                double vy = logScale(-joystick.getLeftX(), DRIVE_DEADBAND, kDrive);
                double omega = logScale(-joystick.getRightX(), ROT_DEADBAND, kRot);

                SmartDashboard.putNumber("VelocityX Setpoint", vx*MaxSpeed);
                SmartDashboard.putNumber("VelocityY Setpoint", vy*MaxSpeed);
                SmartDashboard.putNumber("Angular Setpoint", omega*MaxAngularRate);

                return drive    
                    .withVelocityX(vx * MaxSpeed)
                    .withVelocityY(vy * MaxSpeed)
                    .withRotationalRate(omega * MaxAngularRate);
            })
        );
        
        drivetrain.setDefaultCommand(defaultCommand);

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );


        joystick.a().whileTrue(
            drivetrain.applyRequest(() -> {
                Pose2d start = drivetrain.getStartingPose();
                if (start == null) return drive.withVelocityX(0).withVelocityY(0);

                double[] vel = drivetrain.calculateDriveToPose(start);
                if (vel == null) return brake;

                // smooth out speeds using logarithmic scaling like joysticks
                double vx = logScale(vel[0], DRIVE_DEADBAND, kDrive);
                double vy = logScale(vel[1], DRIVE_DEADBAND, kDrive);
                double o = logScale(vel[2], ROT_DEADBAND, kRot);

                SmartDashboard.putNumber("x", vx);
                SmartDashboard.putNumber("y", vy);
                SmartDashboard.putNumber("r", o);

                // travel to home at 40% speed, rotate at 60% rate
                return drive.withVelocityX(vx * (MaxSpeed * 0.4))
                            .withVelocityY(vy * (MaxSpeed * 0.4))
                            .withRotationalRate(o * (MaxAngularRate * 0.6));
            })
        );

        joystick.b().whileTrue(
            Commands.run(() -> {
                indexer.startShooterFeed();
            }, indexer)
        );

        joystick.x().whileTrue(
            Commands.run(() -> {
                indexer.startHopperIntake();
            }, indexer)
        );

        joystick.rightTrigger(0.1).whileTrue(
            Commands.run(() -> {
                fuel.runUp();
            }, fuel)
        );

        joystick.leftTrigger(0.1).whileTrue(
            Commands.run(() -> {
                fuel.startHopperIntake();
            }, fuel)
        );

        joystick.y().onTrue(
            Commands.runOnce(() -> {
                indexer.stop();
                fuel.stop();
            })
        );

        joystick.povUp().onTrue(
            Commands.runOnce(() -> autoAngleActive = !autoAngleActive, vision)
        );
        joystick.povDown().whileTrue(
            Commands.run(() -> vision.adjustDistance(drivetrain, drive, brake))
        );


        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
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
