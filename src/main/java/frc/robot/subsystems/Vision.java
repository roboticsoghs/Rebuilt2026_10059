package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;

    private double x;
    private double y;
    private double z;
    private double pitch;
    private double yaw;
    private double roll;
    private long aprilTagId;
    private double[] camera = new double[6];

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry cameraPose = table.getEntry("targetpose_cameraspace");

    // Initializing aprilTagId
    public Vision(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        aprilTagId = -1;
    }

    @Override
    public void periodic(){
        table.getEntry("robot_orientation_set").setDoubleArray(
            new double[] {
                drivetrain.getPose().getRotation().getDegrees(),
                0, 0, 0, 0, 0
            }
        );

        camera = cameraPose.getDoubleArray(new double[6]);
        aprilTagId = table.getEntry("tid").getInteger(-1);

        // limelight 3d offset
        x = camera[0];
        y = camera[1];
        z = camera[2];
        pitch = camera[3];
        yaw = camera[4];
        roll = camera[5];

        // SmartDashboard.putNumber("LimelightX", x);
        // SmartDashboard.putNumber("LimelightY", y);
        // SmartDashboard.putNumber("LimelightZ", z);
        // SmartDashboard.putNumber("Limelight Pitch", pitch);
        // SmartDashboard.putNumber("Limelight Yaw", yaw);
        // SmartDashboard.putNumber("Limelight Roll", roll);

        SmartDashboard.putNumber("AprilTag ID", aprilTagId);
        // SmartDashboard.putNumber("Distance to Hub", isAnyAllianceHubFront() ? getZ() : isAnyAllianceHubAnySide() ? getZ() : 0);
        SmartDashboard.putNumber("Distance to Hub", getZ());

        double[] pose = table.getEntry("botpose_orb").getDoubleArray(new double[0]);
        if (pose.length > 0 && table.getEntry("tv").getDouble(0) > 0) {
            Transform2d cameraPose = new Transform2d(
                new Translation2d(0.2286, 0.1143),
                Rotation2d.fromDegrees(180)
            );
            Pose2d visionPose = new Pose2d(pose[0], pose[1], drivetrain.getRotation3d().toRotation2d()).plus(cameraPose);
            double timestamp = Timer.getFPGATimestamp() - (pose[6] / 1000.0);

            drivetrain.addVisionMeasurement(visionPose, timestamp);
        }
    }

    // get values relative to april tag
    public boolean isAprilTag(){
        return x != 0 && z != 0;
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getZ() {
        return z;
    }
    public double getPitch() {
        return pitch;
    }
    public double getYaw() {
        return yaw;
    }
    public double getRoll() {
        return roll;
    }
    public double getId() {
        return aprilTagId;
    }

    public double calculateAprilTagError(double x, double z) {
        if (!isAprilTag()) return 0;

        final double kP = 1.3;

        double xOffset = x - 0.1;
        double zOffset = z;
        double theta = Math.atan2(xOffset, zOffset);
        theta = theta + Math.signum(theta) * Math.toRadians(6);
        double error = 0 - theta;
        error = error * kP;

        SmartDashboard.putNumber("angle error", error);

        return error;
    }

    public double calculateOmegaError() {
        // if (!isAprilTag() || !(isAnyAllianceHubFront() || isAnyAllianceHubAnySide())) return 0.0;
        if (!isAprilTag()) return 0;

        double error = calculateAprilTagError(getX(), getZ());
        return error;
    }

    public void faceAprilTag(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive, SwerveRequest.SwerveDriveBrake brake, double MaxAngularRate) {
        if (!isAprilTag()) return;
        double error = calculateAprilTagError(getX(), getZ());

        drivetrain.setControl(
            drive.withRotationalRate(error * MaxAngularRate)
        );
    }

    public boolean isFacingAprilTag() {
        double error = calculateAprilTagError(getX(), getZ());

        return Math.abs(error) < 0.075;
    }

    public void adjustDistance(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive, SwerveRequest.SwerveDriveBrake brake, double MaxSpeed, double target) {
        if (!isAprilTag()) return;
        double distOffset = getZ();
        double error = target - distOffset;
        double distToMove = -(error * MaxSpeed);

        SmartDashboard.putNumber("dist error", distToMove);

        drivetrain.setControl(
            drive.withVelocityX(distToMove)
        );
    }

    public boolean isAnyAllianceHubFront() {
        return getId() == 10 || getId() == 26;
    }

    public boolean isAnyAllianceHubAnySide() {
        return getId() == 2 || getId() == 18 || getId() == 21 || getId() == 5;
    }
}
