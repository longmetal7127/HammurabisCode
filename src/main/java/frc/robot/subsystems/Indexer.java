package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

/**
 * Indexer subsystem using SparkFlex with NEO Vortex motor.
 * Velocity-controlled flywheel for indexing game pieces.
 */
@Logged(name = "Indexer")
public class Indexer extends SubsystemBase {

  public enum IndexerSetpoint {
    Index(6000),
    Stop(0);

    public final double velocityRPM;

    private IndexerSetpoint(double velocityRPM) {
      this.velocityRPM = velocityRPM;
    }
  }

  // Constants
  private final DCMotor dcMotor = DCMotor.getNeoVortex(1);
  private final int canID = 9;
  private final double gearRatio = 3.0;

  private final double kP = 0.0001;
  private final double kI = 0.0;
  private final double kD = 0.0;
  private final double kV = 0.00009;

  private final double maxVelocityRPM = 6784; // NEO Vortex free speed
  private final double maxAccelerationRPMPerSec = 12000;

  private final int statorCurrentLimit = 60; // Vortex supports higher current

  private final SparkFlex motor;
  private final SparkFlexConfig config;
  private final SparkClosedLoopController pidController;
  private final RelativeEncoder encoder;

  private final SparkSim motorSim;
  private final FlywheelSim indexerSim;

  private double targetVelocityRPM = 0.0;

  @SuppressWarnings("removal")
  public Indexer() {
    motor = new SparkFlex(canID, MotorType.kBrushless);

    pidController = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    config = new SparkFlexConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(statorCurrentLimit)
        .inverted(true);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kP, kI, kD, ClosedLoopSlot.kSlot0);

    config.closedLoop.feedForward
        .kV(kV);

    config.closedLoop.maxMotion
        .maxVelocity(maxVelocityRPM)
        .maxAcceleration(maxAccelerationRPMPerSec);

    config.encoder
        .positionConversionFactor(1.0 / gearRatio)
        .velocityConversionFactor(1.0 / gearRatio);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    motorSim = new SparkSim(motor, dcMotor);
    indexerSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(dcMotor, 0.001, gearRatio),
        dcMotor
    );
  }

  /**
   * Get the current velocity of the indexer in RPM
   * @return Current velocity in RPM
   */
  public double getVelocityRPM() {
    return encoder.getVelocity();
  }

  /**
   * Get the current position of the indexer in rotations
   * @return Current position in rotations
   */
  public double getPositionRotations() {
    return encoder.getPosition();
  }

  /**
   * Get the motor output current
   * @return Current in amps
   */
  public double getOutputCurrent() {
    return motor.getOutputCurrent();
  }

  /**
   * Get the target velocity in RPM
   * @return Target velocity in RPM
   */
  public double getTargetVelocityRPM() {
    return targetVelocityRPM;
  }

  /**
   * Check if the indexer is near the target velocity
   * @param threshold Tolerance in RPM
   * @return True if within threshold of target
   */
  public boolean isNearTarget(double threshold) {
    return Math.abs(getVelocityRPM() - targetVelocityRPM) < threshold;
  }

  /**
   * Drives the indexer to the provided velocity setpoint using MAXMotion.
   *
   * @param setpoint Supplier returning the setpoint to apply
   * @return Command to run
   */
  public Command setTarget(Supplier<IndexerSetpoint> setpoint) {
    return startEnd(() -> {
      IndexerSetpoint target = setpoint.get();
      targetVelocityRPM = target.velocityRPM;
      pidController.setSetpoint(
          8,
          ControlType.kVoltage,
          ClosedLoopSlot.kSlot0
      );
    }, () -> {
      targetVelocityRPM = 0.0;
      motor.stopMotor();
    }).withName("Indexer.setTargetTemporary");
  }
  /**
   * Drives the indexer to the provided velocity setpoint using MAXMotion until the command ends.
   *
   * @param setpoint Supplier returning the setpoint to apply
   * @return Command to run
   */
  public Command setTargetTemporary(Supplier<IndexerSetpoint> setpoint) {
    return startEnd(() -> {
      IndexerSetpoint target = setpoint.get();
      targetVelocityRPM = target.velocityRPM;
      pidController.setSetpoint(
          8,
          ControlType.kVoltage,
          ClosedLoopSlot.kSlot0
      );
    }, () -> {
      targetVelocityRPM = 0.0;
      motor.stopMotor();
    }).withName("Indexer.setTargetTemporary");
  }
  /**
   * Drives the indexer to a specific velocity setpoint.
   *
   * @param setpoint The setpoint to apply
   * @return Command to run
   */
  public Command setTarget(IndexerSetpoint setpoint) {
    return setTarget(() -> setpoint)
        .withName("Indexer.setTarget(" + setpoint.name() + ")");
  }
  public Command setTargetTemporary(IndexerSetpoint setpoint) {
    return setTargetTemporary(() -> setpoint)
        .withName("Indexer.setTargetTemporary(" + setpoint.name() + ")");
  }

  /**
   * Runs the indexer at the index speed.
   *
   * @return Command to run
   */
  public Command index() {
    return setTarget(IndexerSetpoint.Index)
        .withName("Indexer.index");
  }

  /**
   * Stops the indexer.
   *
   * @return Command to run
   */
  public Command stop() {
    return runOnce(() -> {
      targetVelocityRPM = 0.0;
      motor.stopMotor();
    }).withName("Indexer.stop");
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // Convert RPM to rotations per second for simulation
    double velocityRPS = encoder.getVelocity() / 60.0;

    // Update motor sim with current velocity
    motorSim.iterate(velocityRPS * 60, RoboRioSim.getVInVoltage(), 0.02);

    // Update the flywheel simulation with motor voltage
    indexerSim.setInputVoltage(motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    indexerSim.update(0.02);
  }
}
