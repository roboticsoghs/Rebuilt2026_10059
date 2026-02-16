// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FuelSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  
  private boolean isAutoIntakeRunning = false;

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
  
    SmartDashboard.putNumber("FuelSubsystem Motor Temp", m_robotContainer.fuel.motor.getMotorTemperature());
    SmartDashboard.putBoolean("FuelSystem Overheat Warn", m_robotContainer.fuel.motor.getMotorTemperature() >= 70);
  }

  @Override
  public void robotInit() {
    System.out.println("Robot Initialized");
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
    SmartDashboard.putBoolean("TELEOP READY", true);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // Commands.run(() -> m_robotContainer.indexer.startHopperIntake(), m_robotContainer.indexer).onlyIf(() -> m_robotContainer.vision.isEntryTrenchTag()).repeatedly();
    //m_robotContainer.indexer.startHopperIntake()

    
  }

  @Override
  public void teleopPeriodic() {
    Command autointake = Commands.sequence(
      Commands.runOnce(() -> {
        m_robotContainer.fuel.startHopperIntake();
        m_robotContainer.indexer.startHopperIntake();
        isAutoIntakeRunning = true;
      }),
      // Commands.runOnce(() -> {
      //   m_robotContainer.joystick.setRumble(RumbleType.kBothRumble, 1);
      // }),
      Commands.waitSeconds(5),
      Commands.runOnce(() -> {
        m_robotContainer.fuel.stop();
        m_robotContainer.indexer.stop();
        isAutoIntakeRunning = false;
      })
    ).onlyIf(() -> (m_robotContainer.vision.isEntryTrenchTag() && !isAutoIntakeRunning));
    CommandScheduler.getInstance().schedule(autointake);
    // Commands.run(() -> {
    //     m_robotContainer.joystick.setRumble(RumbleType.kBothRumble, 0.5);
    //   });
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
