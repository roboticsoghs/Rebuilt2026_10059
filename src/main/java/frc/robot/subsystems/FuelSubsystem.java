package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelSubsystem extends SubsystemBase {
    private final int motorID = 15;
    private final SparkMax fuel;
    private final SparkMaxConfig config;

    public FuelSubsystem() {
        fuel = new SparkMax(this.motorID, MotorType.kBrushed);
        config = new SparkMaxConfig();

        config.openLoopRampRate(2);
        config.smartCurrentLimit(60);
        config.inverted(true);
        config.idleMode(IdleMode.kCoast);

        fuel.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void startHopperIntake() {
        fuel.set(0.75);
    }

    public void runUp() {
        fuel.set(1.0);
    }

    public void startGroundOuttake() {
        fuel.set(-0.5);
    }

    public void stop() {
        fuel.set(0);
    }
}
