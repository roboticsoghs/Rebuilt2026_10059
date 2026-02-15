package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase{
    private final int motorID = 14;
    private final SparkMax indexer;
    private final SparkMaxConfig config;

    public IndexerSubsystem() {
        indexer = new SparkMax(this.motorID, MotorType.kBrushed);
        config = new SparkMaxConfig();

        config.openLoopRampRate(2);
        config.smartCurrentLimit(40);
        config.idleMode(IdleMode.kCoast);

        indexer.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void startHopperIntake() {
        indexer.set(-0.7);
    }

    public void startShooterFeed() {
        indexer.set(1.0);
    }

    public void startGroundOuttake() {
        indexer.set(0.45);
    }

    public void stop() {
        indexer.stopMotor();
    }
}
