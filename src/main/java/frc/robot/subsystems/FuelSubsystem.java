package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelSubsystem extends SubsystemBase {
    private final int motorID = 15;
    public final SparkMax motor;
    private final SparkMaxConfig config;
    public final SparkClosedLoopController pid;
    public final RelativeEncoder encoder;

    private final double maxAccel = 2700;
    private final int maxVel = 5500;
    public final double allowedError = 0.05;

    private final double SmartVelocityP = 0.0002;
    private final double SmartVelocityI = 0;
    private final double SmartVelocityD = 0.2;

    private final double kS = 0.225; 
    private final double kV = 8.0 / (maxVel * 0.9); // rpm/v
    private final double kA = 0;

    double encoderValue;

    public FuelSubsystem() {
        motor = new SparkMax(this.motorID, MotorType.kBrushless);
        config = new SparkMaxConfig();
        encoderValue = 0;

        pid = motor.getClosedLoopController();
        encoder = motor.getEncoder();

        config.voltageCompensation(12);
        config.smartCurrentLimit(60);
        config.idleMode(IdleMode.kCoast);

        // configure PID
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(SmartVelocityP, SmartVelocityI, SmartVelocityD, ClosedLoopSlot.kSlot0);
        config.closedLoop.maxMotion.maxAcceleration(maxAccel, ClosedLoopSlot.kSlot0);
        config.closedLoop.maxMotion.cruiseVelocity(maxVel, ClosedLoopSlot.kSlot0);
        config.closedLoop.maxMotion.allowedProfileError(allowedError, ClosedLoopSlot.kSlot0);

        // FF
        config.closedLoop.feedForward.kS(kS, ClosedLoopSlot.kSlot0).kV(kV, ClosedLoopSlot.kSlot0).kA(kA, ClosedLoopSlot.kSlot0);

        config.signals.primaryEncoderPositionPeriodMs(5);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Fuel Setpoint", pid.getSetpoint());
        SmartDashboard.putNumber("Fuel Vel", encoder.getVelocity());
    }

    public void startHopperIntake() {
        pid.setSetpoint(0.3 * maxVel, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
    }

    public void runUp() {
        pid.setSetpoint(1.0 * maxVel, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
    }

    public void startGroundOuttake() {
        pid.setSetpoint(-0.6 * maxVel, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
    }

    public void stop() {
        motor.stopMotor();
    }

    /**
     * 
     * @param tolerance The RPM error tolerance
     * @return Returns true when motor is at its setpoint within a tolerance
     */
    public boolean isAtSetpoint(double tolerance) {
        double currentVelocity = encoder.getVelocity();
        double setpoint = pid.getSetpoint();
        double error = Math.abs(currentVelocity - setpoint);
        SmartDashboard.putNumber("curr vel", currentVelocity);
        SmartDashboard.putNumber("setpoint vel", setpoint);
        SmartDashboard.putNumber("error", error);
        return error < tolerance;
    }
}
