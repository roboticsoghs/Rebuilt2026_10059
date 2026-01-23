package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToPoseCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive;
    private final SwerveRequest.SwerveDriveBrake brake;

    private final double distX;
    private final double distY;
    private final Rotation2d distRot;

    private Pose2d targetPose;

    private static final double allowedErrorPos = 0.05; // in meters
    private static final double allowedErrorRot = Math.toRadians(2); // 5 degree tolerance

    private final double maxSpeed;
    private final double maxAngRate;

    public DriveToPoseCommand(
        CommandSwerveDrivetrain drivetrain,
        SwerveRequest.FieldCentric drive,
        SwerveRequest.SwerveDriveBrake brake,
        double dxMeters,
        double dyMeters,
        double drDegrees,
        double MaxSpeed,
        double MaxAngRate
    ) {
        this.drivetrain = drivetrain;
        this.drive = drive;
        this.brake = brake;
        this.distX = dxMeters;
        this.distY = dyMeters;
        this.distRot = Rotation2d.fromDegrees(drDegrees);
        this.maxSpeed = MaxSpeed;
        this.maxAngRate = MaxAngRate;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d start = drivetrain.getPose();

        targetPose = new Pose2d(
            start.getX() + distX,
            start.getY() + distY,
            start.getRotation().plus(distRot)
        );
    }

    @Override
    public void execute() {
        Pose2d curr = drivetrain.getPose();

        double errX = targetPose.getX() - curr.getX();
        double errY = targetPose.getY() - curr.getY();
        double errR = targetPose.getRotation().minus(curr.getRotation()).getRadians();

        double vx = maxSpeed * errX;
        double vy = maxSpeed * errY;
        double vr = maxAngRate * errR;

        drivetrain.setControl(
            drive.withVelocityX(vx)
                 .withVelocityY(vy)
                 .withRotationalRate(vr)
        );
    }

    @Override
    public boolean isFinished() {
        Pose2d curr = drivetrain.getPose();

        boolean posDone = Math.hypot(targetPose.getX() - curr.getX(), targetPose.getY() - curr.getY()) < allowedErrorPos;

        SmartDashboard.putBoolean("Drive To Pose finished", posDone);

        return posDone;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(brake);
    }
}