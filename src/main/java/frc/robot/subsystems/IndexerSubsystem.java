package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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
        config.smartCurrentLimit(60);
        config.idleMode(IdleMode.kCoast);

        indexer.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void spinUp(double setpoint) {
        indexer.set(setpoint);
    }

    public void stop() {
        indexer.set(0);
    }
}
