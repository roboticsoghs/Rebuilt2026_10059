// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  // teleop times
  private static final double TRANSITION_PERIOD = 10.0;
  private static final double ACTIVE_SHIFT = 25.0;
  private static final double ENDGAME_PERIOD = 30.0;

  private static final double[] phaseDurations = {TRANSITION_PERIOD, ACTIVE_SHIFT, ACTIVE_SHIFT, ACTIVE_SHIFT, ACTIVE_SHIFT, ENDGAME_PERIOD};

  private double teleopStart;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Robot Diagnostics
    SmartDashboard.putBoolean("ROBOT READY", DriverStation.isJoystickConnected(0) && DriverStation.isDSAttached());
    SmartDashboard.putBoolean("FMS CONNECTED", DriverStation.isFMSAttached());

    SmartDashboard.putNumber("Robot VelocityX", m_robotContainer.drivetrain.getState().Speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Robot VelocityY", m_robotContainer.drivetrain.getState().Speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Robot Omega", m_robotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond);

    SmartDashboard.putNumber("FuelSubsystem Motor Temp", ((m_robotContainer.fuel.motor1.getMotorTemperature() + m_robotContainer.fuel.motor2.getMotorTemperature()) / 2));
    SmartDashboard.putBoolean("FuelSystem Overheat Warn", ((m_robotContainer.fuel.motor1.getMotorTemperature() + m_robotContainer.fuel.motor2.getMotorTemperature()) / 2) >= 55);
    SmartDashboard.putNumber("Indexer Motor Temp", m_robotContainer.indexer.motor.getMotorTemperature());
    SmartDashboard.putBoolean("Indexer Overheat Warn", m_robotContainer.indexer.motor.getMotorTemperature() >= 60);
    SmartDashboard.putBoolean("Shooter Ready", m_robotContainer.fuel.isAtSetpoint(100));
    SmartDashboard.putBoolean("AimAssist Available", m_robotContainer.vision.isAprilTag() && (m_robotContainer.vision.isAnyAllianceHubFront() || m_robotContainer.vision.isAnyAllianceHubAnySide()));
  }

  @Override  public void robotInit() {
    System.out.println("Robot Initialized");

    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5800, "limelight.local", 5800);
  }


  @Override
  public void disabledInit() {
    SmartDashboard.putBoolean("AUTO READY", false);
    SmartDashboard.putBoolean("TELEOP READY", false);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    SmartDashboard.putBoolean("AUTO READY", true);
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putBoolean("AUTO FINISHED", m_autonomousCommand.isFinished());
  }

  @Override
  public void autonomousExit() {
    SmartDashboard.putBoolean("AUTO FINISHED", true);
  }

  @Override
  public void teleopInit() {
    // m_robotContainer.fuel.stop();
    // m_robotContainer.indexer.stop();
    teleopStart = Timer.getFPGATimestamp();
    SmartDashboard.putBoolean("TELEOP READY", true);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    double elapsed = Timer.getFPGATimestamp() - teleopStart;

    double sum = 0;
    double remaining = 0;

    for (double duration : phaseDurations) {
      sum += duration;
      if (elapsed < sum) {
        remaining = sum - elapsed;
        break;
      }
    }

    SmartDashboard.putNumber("PHASE TIME", remaining);
    SmartDashboard.putNumber("MATCH TIME", elapsed);
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
