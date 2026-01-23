// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // drivetrain IDs
    public static int leftFrontMotorPort = 1;
    public static int leftBackMotorPort = 4;
    public static int rightFrontMotorPort = 2;
    public static int rightBackMotorPort = 5;

    // elevator IDs
    public static int elevatorRightMotorPort = 6;
    public static int elevatorLeftMotorPort = 7;
    public static int limitPort = 9;
    public static int limitCoralPort = 8;
    public static int shooterPort = 10;

    // Configured these values in inches
    public static int LEVEL_0_HEIGHT = 8;
    public static int LEVEL_1_HEIGHT = 8;
    public static int LEVEL_2_HEIGHT = 8;
    public static int LEVEL_3_HEIGHT = 8;

    public static int UPPER_HARD_LIMIT = 50;
    public static int UPPER_LOWER_LIMIT = -2;

    // operator board
    public static int SLOWMODE_PORT = 9;
    public static int LEFT_ALIGN_PORT = 6;
    public static int RIGHT_ALIGN_PORT = 7;
    // public static int ABORT_PORT = 8;
    public static int CENTER_ALIGN_PORT = 1;
    public static int UNSTUCK_CORAL_PORT = 2;

    // elevator
    public static double ElevatorgearRatio = 12;
    public static double ElevatorwheelRadius = 0.875;

    // drive train
    public static double DriveTrainGearRatio = 8.714;
    public static double DriveTrainWheelRadius = 3; // inches

    public static int encoderTicksPerRotation = 42;

    public static double offSetHeight = 3.5;
    public static boolean slowMode = false;
    public static double slowModeMultipler = 0.3;

    // inches
    // TODO: adjust constants
    public static double leftAlignReef = -5.0; // prev: 
    public static double reefCenter = -9.5; // prev: -9.5
    public static double rightAlignReef = -15.0; // prev: -15

    // allowed error for motors
    public static double allowedError = 0.05;

    // other values
    public static int leftAxis = 1;
    public static int rightAxis = 5;
}