package frc.robot.subsystems;

import org.opencv.photo.AlignMTB;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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

    // Initializing da alignState and aprilTagId
    public Vision() {
        aprilTagId = 0;
    }

    @Override
    public void periodic(){
        // tis method gets called every scheduler run
        // read da value periodically
        double[] defaultValue = {0,0,0,0,0,0};
        camera = cameraPose.getDoubleArray(defaultValue);
        long defaultValueId = 0;
        aprilTagId = table.getEntry("tid").getInteger(defaultValueId);

        // limelight 3d offset
        x = camera[0];
        y = camera[1];
        z = camera[2];
        pitch = camera[3];
        yaw = camera[4];
        roll = camera[5];

        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightZ", z);

        SmartDashboard.putNumber("Limelight Pitch", pitch);
        SmartDashboard.putNumber("Limelight Yaw", yaw);
        SmartDashboard.putNumber("Limelight Roll", roll);

        SmartDashboard.putNumber("AprilTag ID", aprilTagId);
    }

    // Get them values yo (eggs)
    public boolean isAprilSeen(){
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

    public void faceAprilTag(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive, SwerveRequest.SwerveDriveBrake brake) {
        double xOffset = getX();
        double zOffset = getZ();
        double theta = Math.atan(xOffset / zOffset);
        double error = 0 - theta;

        drivetrain.setControl(
            drive.withRotationalRate(error)
        );
    }

    public void adjustDistance(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive, SwerveRequest.SwerveDriveBrake brake) {
        double distOffset = getZ();
        double error = 4.75 - distOffset;

        drivetrain.setControl(
            drive.withVelocityY(error)
        );
    }

    public boolean isChuteTag() {
        return getId() == 10 || getId() == 26;
    }
}
