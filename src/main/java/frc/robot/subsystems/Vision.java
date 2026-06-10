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
        // table.getEntry("robot_orientation_set").setDoubleArray(
        //     new double[] {
        //         drivetrain.getPose().getRotation().getDegrees(),
        //         0, 0, 0, 0, 0
        //     }
        // );

        camera = cameraPose.getDoubleArray(new double[6]);
        aprilTagId = table.getEntry("tid").getInteger(-1);

        // limelight 3d offset
        x = camera[0];
        y = camera[1];
        z = camera[2];
        roll = camera[3];
        pitch = camera[4];
        yaw = camera[5];

        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightZ", z);
        SmartDashboard.putNumber("Limelight Pitch", pitch);
        SmartDashboard.putNumber("Limelight Yaw", yaw);
        SmartDashboard.putNumber("Limelight Roll", roll);

        SmartDashboard.putNumber("AprilTag ID", aprilTagId);
        // SmartDashboard.putNumber("Distance to Hub", isAnyAllianceHubFront() ? getZ() : isAnyAllianceHubAnySide() ? getZ() : 0);
        SmartDashboard.putNumber("Distance to Hub", getZ()); // debug
        SmartDashboard.putNumber("Omega Error", calculateYawError(yaw));

        // double[] pose = table.getEntry("botpose_orb").getDoubleArray(new double[0]);
        // if (pose.length > 0 && table.getEntry("tv").getDouble(0) > 0) {
        //     Pose2d visionPose = new Pose2d(pose[0], pose[1], drivetrain.getRotation3d().toRotation2d());
        //     double timestamp = Timer.getFPGATimestamp() - (pose[6] / 1000.0);

        //     drivetrain.addVisionMeasurement(visionPose, timestamp);
        // }
    }

    // get values relative to april tag
    public boolean isAprilTag(){
        return table.getEntry("tv").getDouble(0) == 1;
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
    public long getId() {
        return aprilTagId;
    }

    public double calculateYawError() {
        if (!isAprilTag()) return 0;

        final double kP = 0.06;
        double error = -getYaw(); // same as subtracting from zero
        
        // deadband
        if (Math.abs(error) < 1.0) return 0;

        return kP * error;
    }

    // TODO: remove
    public double calculateAprilTagError(double x, double z) {
        if (!isAprilTag()) return 0;

        final double kP = 0.6;

        double xOffset = x - 0.05;
        double zOffset = z;
        double theta = Math.atan2(xOffset, zOffset);
        theta = theta * Math.signum(theta);
        double error = 0 - theta;
        error = error * kP;

        SmartDashboard.putNumber("angle error", error);

        return error;
    }

    // TODO: remove
    public double calculateOmegaError() {
        if (!isAprilTag()) return 0;

        double error = calculateAprilTagError(getX(), getZ());

        if (Math.abs(error) < 0.7) return 0.0;

        return error;
    }

    // TODO: use new method
    public void faceAprilTag(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive, SwerveRequest.SwerveDriveBrake brake, double MaxAngularRate) {
        if (!isAprilTag()) return;
        double error = calculateAprilTagError(getX(), getZ());

        drivetrain.setControl(
            drive.withRotationalRate(error * MaxAngularRate)
        );
    }

    // TODO: use new method
    public boolean isFacingAprilTag() {
        double error = calculateAprilTagError(getX(), getZ());

        return Math.abs(error) < 0.075;
    }

    // TODO: remove
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

    public double calculateDistanceError(double targetMeters) {
        if (!isAprilTag()) return 0;

        final double kP = 1.0;
        double error = targetMeters - getZ();

        if (Math.abs(error) < 0.05) return 0;

        return kP * error;
    }

    public double calculateHorizontalError() {
        if (!isAprilTag()) return 0;

        final double kP = 1.5;
        double error = -getX();

        if (Math.abs(error) < 0.04) return 0;

        return kP * error;
    }
    
    public boolean isAnyAllianceHubFront() {
        return getId() == 10 || getId() == 26;
    }

    public boolean isAnyAllianceHubAnySide() {
        return getId() == 2 || getId() == 18 || getId() == 21 || getId() == 5;
    }
}
