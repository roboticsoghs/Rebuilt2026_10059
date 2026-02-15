package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase{
    private final int motorID = 14;
    public final SparkMax Motor;
    private final SparkMaxConfig config;
    private final SparkClosedLoopController pid;
    private final RelativeEncoder encoder;

    private final double SmartVelocityP = 0.55;
    private final double SmartVelocityI = 0;
    private final double SmartVelocityD = 0;

    private final double maxAccel = 2700;
    private final int maxVel = 3500;
    public final double allowedError = 0.05;

    double encoderValue;

    public IndexerSubsystem() {
        Motor = new SparkMax(this.motorID, MotorType.kBrushless);
        config = new SparkMaxConfig();
        encoderValue = 0;

        pid = Motor.getClosedLoopController();
        encoder = Motor.getEncoder();

        config.voltageCompensation(12);
        config.smartCurrentLimit(60);
        config.idleMode(IdleMode.kCoast);

        // configure PID
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(SmartVelocityP, SmartVelocityI, SmartVelocityD, ClosedLoopSlot.kSlot0);
        config.closedLoop.maxMotion.maxAcceleration(maxAccel, ClosedLoopSlot.kSlot0);
        config.closedLoop.maxMotion.cruiseVelocity(maxVel, ClosedLoopSlot.kSlot0);
        config.closedLoop.maxMotion.allowedProfileError(allowedError, ClosedLoopSlot.kSlot0);

        config.signals.primaryEncoderPositionPeriodMs(5);
        Motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void startHopperIntake() {
        pid.setSetpoint(-0.7 * maxVel, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
    }

    public void startShooterFeed() {
        pid.setSetpoint(1.0 * maxVel, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
    }

    public void startGroundOuttake() {
        pid.setSetpoint(0.45 * maxVel, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
    }

    public void stop() {
        Motor.stopMotor();
    }
}
