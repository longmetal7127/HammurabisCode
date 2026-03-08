package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

/**
 * Spindexer subsystem using SparkMax with NEO motor.
 * Velocity-controlled flywheel for spinning/agitating game pieces.
 */
@Logged(name = "Spindexer")
public class Spindexer extends SubsystemBase {

  public enum SpindexerSetpoint {
    Spin(3000),
    Stop(0);

    public final double velocityRPM;

    private SpindexerSetpoint(double velocityRPM) {
      this.velocityRPM = velocityRPM;
    }
  }

  // Constants
  private final DCMotor dcMotor = DCMotor.getNEO(1);
  private final int canID = 8;
  private final double gearRatio = 3.0;

  private final double kP = 0.00091287;
  private final double kI = 0.0;
  private final double kD = 0.0;
  private final double kV = 0.0060921;
  private final double kA = 0.00068446;
  private final double kS = 0.083021;

  private final double maxVelocityRPM = 5676; // NEO free speed
  private final double maxAccelerationRPMPerSec = 10000;

  private final int statorCurrentLimit = 40;

  private final SparkMax motor = new SparkMax(canID, MotorType.kBrushless);;
  private final SparkMaxConfig config;
  private final SparkClosedLoopController pidController;
  private final RelativeEncoder encoder;

  private final SparkSim motorSim;
  private final FlywheelSim spindexerSim;

  private double targetVelocityRPM = 0.0;
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
          null // Use default timeout (10 s)
               // Log state with Phoenix SignalLogger class
          
      ),
      new SysIdRoutine.Mechanism(
          (volts) -> {
            motor.setVoltage((volts.in(Volts)));
          },
          null,
          this));

  @SuppressWarnings("removal")
  public Spindexer() {

    pidController = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(statorCurrentLimit)
        .inverted(false);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kP, kI, kD, ClosedLoopSlot.kSlot0);

    config.closedLoop.feedForward
        .kV(kV)
        .kS(kS)
        .kA(kA);

    config.closedLoop.maxMotion
        .maxAcceleration(maxAccelerationRPMPerSec);

    config.encoder
        .positionConversionFactor(1.0 / gearRatio)
        .velocityConversionFactor(1.0 / gearRatio);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    motorSim = new SparkSim(motor, dcMotor);
    spindexerSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(dcMotor, 0.001, gearRatio),
        dcMotor);
  }

  /**
   * Get the current velocity of the spindexer in RPM
   * 
   * @return Current velocity in RPM
   */
  public double getVelocityRPM() {
    return encoder.getVelocity();
  }

  public double getVoltage() {
    return motor.getAppliedOutput() * motor.getBusVoltage();
  }

  /**
   * Get the current position of the spindexer in rotations
   * 
   * @return Current position in rotations
   */
  public double getPositionRotations() {
    return encoder.getPosition();
  }

  /**
   * Get the motor output current
   * 
   * @return Current in amps
   */
  public double getOutputCurrent() {
    return motor.getOutputCurrent();
  }

  /**
   * Get the target velocity in RPM
   * 
   * @return Target velocity in RPM
   */
  public double getTargetVelocityRPM() {
    return targetVelocityRPM;
  }

  /**
   * Check if the spindexer is near the target velocity
   * 
   * @param threshold Tolerance in RPM
   * @return True if within threshold of target
   */
  public boolean isNearTarget(double threshold) {
    return Math.abs(getVelocityRPM() - targetVelocityRPM) < threshold;
  }

  /**
   * Drives the spindexer to the provided velocity setpoint using MAXMotion.
   *
   * @param setpoint Supplier returning the setpoint to apply
   * @return Command to run
   */
  public Command setTarget(Supplier<SpindexerSetpoint> setpoint) {
    return run(() -> {
      SpindexerSetpoint target = setpoint.get();
      targetVelocityRPM = target.velocityRPM;
      pidController.setSetpoint(
          targetVelocityRPM,
          ControlType.kMAXMotionVelocityControl,
          ClosedLoopSlot.kSlot0);
    }).withName("Spindexer.setTarget");
  }

  /**
   * Drives the spindexer to the provided velocity setpoint using MAXMotion.
   *
   * @param setpoint Supplier returning the setpoint to apply
   * @return Command to run
   */
  public Command setTargetTemporary(Supplier<SpindexerSetpoint> setpoint) {
    return startEnd(() -> {
      SpindexerSetpoint target = setpoint.get();
      targetVelocityRPM = target.velocityRPM;
      pidController.setSetpoint(
          8,
          ControlType.kVoltage,
          ClosedLoopSlot.kSlot0);
    }, () -> {
      targetVelocityRPM = 0.0;
      motor.stopMotor();
    }).withName("Spindexer.setTargetTemporary");
  }

  /**
   * Drives the spindexer to a specific velocity setpoint.
   *
   * @param setpoint The setpoint to apply
   * @return Command to run
   */
  public Command setTarget(SpindexerSetpoint setpoint) {
    return setTarget(() -> setpoint)
        .withName("Spindexer.setTarget(" + setpoint.name() + ")");
  }

  /**
   * Drives the spindexer to a specific velocity setpoint.
   *
   * @param setpoint The setpoint to apply
   * @return Command to run
   */
  public Command setTargetTemporary(SpindexerSetpoint setpoint) {
    return setTargetTemporary(() -> setpoint)
        .withName("Spindexer.setTargetTemporary(" + setpoint.name() + ")");
  }

  /**
   * Runs the spindexer at the spin speed.
   *
   * @return Command to run
   */
  public Command spin() {
    return setTarget(SpindexerSetpoint.Spin)
        .withName("Spindexer.spin");
  }

  /**
   * Stops the spindexer.
   *
   * @return Command to run
   */
  public Command stop() {
    return runOnce(() -> {
      targetVelocityRPM = 0.0;
      motor.stopMotor();
    }).withName("Spindexer.stop");
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
    spindexerSim.setInputVoltage(motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    spindexerSim.update(0.02);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
