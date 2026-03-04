package frc.robot.subsystems;

import org.opencv.photo.AlignMTB;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {
    // limelight variables and stuff
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

    Pose2d botpose;
    double timestamp;

    // Initializing da alignState and aprilTagId
    public Vision() {
        aprilTagId = -1;
    }

    @Override
    public void periodic(){
        // tis method gets called every scheduler run
        // read da value periodically
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
        SmartDashboard.putNumber("Distance to Hub", isAnyAllianceHubFront() ? getZ() : isAnyAllianceHubAnySide() ? getZ() : 0);
    
        double[] pose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        if (pose.length > 0 && table.getEntry("tv").getDouble(0) > 0) {
            botpose = new Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[5]));
            timestamp = Timer.getFPGATimestamp() - (pose[6] / 1000.0);
        }
    }

    public Pose2d getPose() {
        return botpose;
    }

    public double getTimestamp() {
        return timestamp;
    }

    // Add these imports
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.Timer;

// // Inside Vision class periodic()
// public void periodic() {
//     // botpose_wpiblue: [x, y, z, roll, pitch, yaw, totalLatency, tagCount, tagSpan, avgTagDist, avgTagArea]
//     double[] botpose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);

//     if (botpose.length > 0 && table.getEntry("tv").getDouble(0) > 0) {
//         Pose2d pose = new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));
        
//         // Calculate timestamp: Current time minus total latency (converted to seconds)
//         double timestamp = Timer.getFPGATimestamp() - (botpose[6] / 1000.0);
        
//         // Feed to drivetrain (assuming you pass drivetrain into Vision or use a static reference)
//         RobotContainer.drivetrain.addVisionMeasurement(pose, timestamp);
//     }
// }


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

    public void faceAprilTag(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive, SwerveRequest.SwerveDriveBrake brake, double MaxAngularRate) {
        if (!isAprilTag()) return;
        double xOffset = getX() - 0.1;
        double zOffset = getZ();
        double theta = Math.atan2(xOffset, zOffset);
        double error = 0 - theta;
        error = error * 1.5;

        drivetrain.setControl(
            drive.withRotationalRate(error * MaxAngularRate)
        );
    }

    public boolean isFacingAprilTag(double additionalOffset) {
        double xOffset = getX() - 0.1;
        double zOffset = getZ();
        double theta = Math.atan2(xOffset, zOffset) + additionalOffset;
        double error = 0 - theta;
        error = error * 1.5;

        return Math.abs(error) < 0.1;
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
        return getId() == 10 || getId() == 26 || getId() == 7; // temp: 7
    }

    public boolean isAnyAllianceHubAnySide() {
        return getId() == 2 || getId() == 18 || getId() == 21 || getId() == 5 || getId() == 7; //temp: 7
    }
}
