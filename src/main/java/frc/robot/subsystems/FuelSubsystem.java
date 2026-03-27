package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelSubsystem extends SubsystemBase {
    private final double MAX_ALLOWED_SPEED = 0.8;

    private final int motor1ID = 15;
    private final int motor2ID = 16;

    // For motor1
    public final SparkMax motor1;
    private final SparkMaxConfig config1;
    public final SparkClosedLoopController pid1;
    public final RelativeEncoder encoder1;

    // For motor2
    public final SparkMax motor2;
    private final SparkMaxConfig config2;
    public final SparkClosedLoopController pid2;
    public final RelativeEncoder encoder2;

    // Settings                                                                                                                      a
    private final double maxAccel = 5000;
    private final int maxVel = 5300;
    public final double allowedError = 0.05;

    private final double SmartVelocityP = 0.0003;
    private final double SmartVelocityI = 0.0;
    private final double SmartVelocityD = 0.01;

    private final double SmartVelocityP2 = 0.0002;
    private final double SmartVelocityI2 = 0.0;
    private final double SmartVelocityD2 = 0.2;

    private final double kS = 0.225;
    private final double kV = 11.0 / maxVel; // v/rpm
    private final double kA = 0;


    public FuelSubsystem() {
        motor1 = new SparkMax(this.motor1ID, MotorType.kBrushless);
        motor2 = new SparkMax(this.motor2ID, MotorType.kBrushless);


        config1 = new SparkMaxConfig();
        config2 = new SparkMaxConfig();

        pid1 = motor1.getClosedLoopController();
        pid2 = motor2.getClosedLoopController();


        encoder1 = motor1.getEncoder();
        encoder2 = motor2.getEncoder();

        config1.voltageCompensation(12);
        config1.smartCurrentLimit(60);
        config1.idleMode(IdleMode.kCoast);

        // motor 1 config
        // configure PID slot 0 (kVelocity)
        config1.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config1.closedLoop.pid(SmartVelocityP, SmartVelocityI, SmartVelocityD, ClosedLoopSlot.kSlot0);
        // config.closedLoop.maxMotion.maxAcceleration(maxAccel, ClosedLoopSlot.kSlot0);
        config1.closedLoop.maxMotion.cruiseVelocity(maxVel, ClosedLoopSlot.kSlot0);
        config1.closedLoop.maxMotion.allowedProfileError(allowedError, ClosedLoopSlot.kSlot0);

        // configure PID slot 1 (MaxMotion)
        config1.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config1.closedLoop.pid(SmartVelocityP2, SmartVelocityI2, SmartVelocityD2, ClosedLoopSlot.kSlot1);
        config1.closedLoop.maxMotion.maxAcceleration(maxAccel, ClosedLoopSlot.kSlot1);
        config1.closedLoop.maxMotion.cruiseVelocity(maxVel, ClosedLoopSlot.kSlot1);
        config1.closedLoop.maxMotion.allowedProfileError(allowedError, ClosedLoopSlot.kSlot1);

        // // FF
        config1.closedLoop.feedForward.kS(kS, ClosedLoopSlot.kSlot0).kV(kV, ClosedLoopSlot.kSlot0).kA(kA, ClosedLoopSlot.kSlot0);
        config1.closedLoop.feedForward.kS(kS, ClosedLoopSlot.kSlot1).kV(kV, ClosedLoopSlot.kSlot1).kA(kA, ClosedLoopSlot.kSlot1);

        // motor 2 config
        config2.voltageCompensation(12);
        config2.smartCurrentLimit(60);
        config2.idleMode(IdleMode.kCoast);

        config2.follow(motor1, false);

        config1.signals.primaryEncoderPositionPeriodMs(5);
        motor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config2.signals.primaryEncoderPositionPeriodMs(5);
        motor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Fuel Setpoint", pid1.getSetpoint());
        SmartDashboard.putNumber("Fuel Vel", encoder1.getVelocity());
        SmartDashboard.putNumber("Fuel2 Vel", encoder2.getVelocity());
        SmartDashboard.putNumber("fuel voltage", motor1.getAppliedOutput() * motor1.getBusVoltage());
    }

    public void startHopperIntake() {
        pid1.setSetpoint(0.5 * maxVel, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
    }

    public void runUp(double speed) {
        pid1.setSetpoint(speed * maxVel, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public void startGroundOuttake() {
        pid1.setSetpoint(-0.6 * maxVel, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
    }

    public void stop() {
        motor1.stopMotor();
        motor2.stopMotor();
    }

    /**
     *
     * @param tolerance The RPM error tolerance
     * @return Returns true when motor is at its setpoint within a tolerance
     */
    public boolean isAtSetpoint(double tolerance) {
        double currentVelocity = encoder1.getVelocity();
        double setpoint = pid1.getSetpoint();
        double error = Math.abs(currentVelocity - setpoint);
        SmartDashboard.putNumber("curr vel", currentVelocity);
        SmartDashboard.putNumber("setpoint vel", setpoint);
        SmartDashboard.putNumber("error", error);
        return error < tolerance;
    }

    public double calcSpeedByDistance(double dist) {
        if (dist == 0) return MAX_ALLOWED_SPEED;
        double speed = 0.0118029*Math.pow(dist, 2) + 0.00907623*dist + 0.596337;
        speed *= 0.95;
        speed = Math.min(speed, MAX_ALLOWED_SPEED);
        SmartDashboard.putNumber("auto speed", speed);
        return speed;
    }
}
