package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;

import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TiltedElevatorSim;

/**
 * Lintake subsystem using SparkFlex with NEO motor
 */
@Logged(name = "Lintake")
public class Lintake extends SubsystemBase {

  // Constants
  private final DCMotor dcMotor = DCMotor.getNeoVortex(1);
  private final int canID = 6;
  private final double gearRatio = 5;
  private final double kP = 3;
  private final double kI = 0;
  private final double kD = 0.01;
  private final double kS = 0;
  private final double kV = 6.61;
  private final double kA = 0.04;
  private final double kG = 0;
  private final double maxVelocity = 8;
  private final double maxAcceleration = 200;
  private final boolean brakeMode = false;
  private final int statorCurrentLimit = 40;
  private final double drumRadius = Units.inchesToMeters(0.5); // meters
  private final double minHeight = 0;
  private final double maxHeight = 0.318;

  // Feedforward

  // Motor controller
  private final SparkFlex motor;
  private final RelativeEncoder encoder;
  private final SparkSim motorSim;
  private final SparkClosedLoopController sparkPidController;

  private final SparkMax intakeWheelMotor;
  private final SparkClosedLoopController sparkWheelPidController;

  // Simulation
  private final TiltedElevatorSim intakeSim;

  /**
   * Creates a new Lintake Subsystem.
   */
  public Lintake() {
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motor = new SparkFlex(canID, MotorType.kBrushless);
    motorConfig.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

    encoder = motor.getEncoder();
    encoder.setPosition(0);

    motorConfig.smartCurrentLimit(statorCurrentLimit);

    sparkPidController = motor.getClosedLoopController();
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kP, kI, kD, ClosedLoopSlot.kSlot0).outputRange(-1, 1);
    motorConfig.closedLoop.feedForward.kS(kS).kV(kV).kA(kA);
    motorConfig.closedLoop.feedForward.kG(kG);
    motorConfig.closedLoop.maxMotion.cruiseVelocity(maxVelocity);
    motorConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal).allowedProfileError(0.02)
        .maxAcceleration(maxAcceleration);

    // Conversion: motor rotations -> meters, motor RPM -> m/s
    double metersPerMotorRotation = (2.0 * Math.PI * drumRadius) / gearRatio;
    motorConfig.encoder
        .positionConversionFactor(metersPerMotorRotation)
        .velocityConversionFactor(metersPerMotorRotation / 60.0);

    motorConfig.inverted(true);
    motorConfig.softLimit.forwardSoftLimit(0.317).forwardSoftLimitEnabled(true);
    motor.configure(
        motorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    motorSim = new SparkSim(motor, dcMotor);

    // Initialize simulation
    intakeSim = new TiltedElevatorSim(
        dcMotor,
        gearRatio,
        Units.lbsToKilograms(12.58),
        drumRadius,
        0,
        1,
        true,
        0);

    intakeWheelMotor = new SparkMax(7, MotorType.kBrushless);
    sparkWheelPidController = intakeWheelMotor.getClosedLoopController();
    SparkFlexConfig wheelConfig = new SparkFlexConfig();
    wheelConfig.idleMode(IdleMode.kBrake);
    wheelConfig.smartCurrentLimit(48);
    wheelConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.1, 0, 0, ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1);

    intakeWheelMotor.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Update simulation and telemetry.
   */
  @Override
  public void periodic() {
  }

  /**
   * Update simulation.
   */
  @Override
  public void simulationPeriodic() {
    // Use getVoltage() for other controllers
    intakeSim.setInput(getVoltage());

    // Update simulation by 20ms
    intakeSim.update(0.020);

    motorSim.iterate(intakeSim.getVelocityMetersPerSecond(), 12, 0.02);
  }

  /**
   * Get the current position in meters.
   * 
   * @return Position in meters
   */
  @Logged(name = "Position/Meters")
  public double getPosition() {
    // Encoder already returns meters via conversion factor
    return encoder.getPosition();
  }

  /**
   * Get the current velocity in meters per second.
   * 
   * @return Velocity in meters per second
   */
  @Logged(name = "Velocity")
  public double getVelocity() {
    // Encoder already returns m/s via conversion factor
    return encoder.getVelocity();
  }

  /**
   * Get the current applied voltage.
   * 
   * @return Applied voltage
   */
  @Logged(name = "Voltage")
  public double getVoltage() {
    return motor.getAppliedOutput() * motor.getBusVoltage();
  }

  /**
   * Get the current motor current.
   * 
   * @return Motor current in amps
   */
  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  /**
   * Get the current motor temperature.
   * 
   * @return Motor temperature in Celsius
   */
  public double getTemperature() {
    return motor.getMotorTemperature();
  }

  /**
   * Set intake position.
   * 
   * @param position The target position in meters
   */
  public void setPosition(double position) {
    setPosition(position, 0);
  }

  /**
   * Set intake position with acceleration.
   * 
   * @param position     The target position in meters
   * @param acceleration The acceleration in meters per second squared
   */
  public void setPosition(double position, double acceleration) {
    // Encoder units are already meters, pass directly
    sparkPidController.setSetpoint(
        position,
        ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0);
  }

  /**
   * Set intake velocity.
   * 
   * @param velocity The target velocity in meters per second
   */
  public void setVelocity(double velocity) {
    setVelocity(velocity, 0);
  }

  public Command setVelocityCommand(double velocity) {
    return startEnd(() -> setVelocity(velocity), () -> setVelocity(0))
        .withName("Lintake.setVelocityCommand(" + velocity + " m/s)");
  }

  /**
   * Set intake velocity with acceleration.
   * 
   * @param velocity     The target velocity in meters per second
   * @param acceleration The acceleration in meters per second squared
   */
  public void setVelocity(double velocity, double acceleration) {
    // Encoder units are already m/s, pass directly
    sparkWheelPidController.setSetpoint(
        velocity * 12,
        ControlType.kVoltage,
        ClosedLoopSlot.kSlot0);
  }

  /**
   * Set motor voltage directly.
   * 
   * @param voltage The voltage to apply
   */
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /**
   * Get the intake simulation for testing.
   * 
   * @return The intake simulation model
   */
  public TiltedElevatorSim getSimulation() {
    return intakeSim;
  }

  public double getMinHeightMeters() {
    return minHeight;
  }

  public double getMaxHeightMeters() {
    return maxHeight;
  }

  /**
   * Creates a command to set the intake to a specific height.
   * 
   * @param heightMeters The target height in meters
   * @return A command that sets the intake to the specified height
   */
  public Command setHeightCommand(double heightMeters) {
    return runOnce(() -> setPosition(heightMeters));
  }

  /**
   * Creates a command to stop the intake.
   * 
   * @return A command that stops the intake
   */
  public Command stopCommand() {
    return runOnce(() -> setVelocity(0));
  }

  /**
   * Creates a command to move the intake at a specific velocity.
   * 
   * @param velocityMetersPerSecond The target velocity in meters per second
   * @return A command that moves the intake at the specified velocity
   */
  public Command moveAtVelocityCommand(double velocityMetersPerSecond) {
    return run(() -> setVelocity(velocityMetersPerSecond));
  }

  /**
   * Gets the 3D pose of the intake mechanism for visualization.
   * The intake is tilted 14.478 degrees below horizontal and extends out the
   * front of the robot.
   * 
   * @return Pose3d representing the intake's position and orientation
   */
  public Pose3d getMechanismPose() {
    double currentHeightMeters = getPosition(); // already in meters
    double tiltAngleDegrees = -14.478;

    // Create pose with translation and rotation
    // Assuming intake base is at front center of robot
    return new Pose3d(
        new Translation3d(-currentHeightMeters, 0, 0)
            .rotateBy(new Rotation3d(0, Math.toRadians(tiltAngleDegrees), Math.PI)), // Base
        // position
        // +
        // extension
        new Rotation3d(Math.PI * 0.5, 0, Math.PI));
  }

  public boolean getIntakeEnabled() {
    return MathUtil.isNear(getPosition(), maxHeight, 0.02);
  }

  public Command zeroingRoutine() {
    return run(() -> {
      motor.set(-0.25);
      runReverse();
    }).until(() -> (getCurrent() >= 35)).andThen(() -> {
      encoder.setPosition(0);
      motor.set(0);
      setVelocity(0);
    });
  }

  public Command runReverse() {
    return Commands.runOnce(() -> {
      setVelocity(-.1);
    });
  }

  public Command deployLintake() {
    return Commands.sequence(
        setHeightCommand(getMaxHeightMeters()),
        Commands.waitUntil(() -> (Math.abs(getPosition() - getMaxHeightMeters()) <= 0.02)),
        runReverse()
        );
  }

  /**
   * Creates a command that oscillates the intake continuously using a sinusoidal
   * setpoint.
   *
   * @param lowerPosition The lower position bound in meters
   * @param upperPosition The upper position bound in meters
   * @param frequencyHz   Oscillation frequency in Hz
   * @return A command that continuously applies a time-parameterized sinusoidal
   *         setpoint
   */
  public Command oscillateIntake(double lowerPosition, double upperPosition, double frequencyHz) {
    double boundedLower = MathUtil.clamp(Math.min(lowerPosition, upperPosition), minHeight, maxHeight);
    double boundedUpper = MathUtil.clamp(Math.max(lowerPosition, upperPosition), minHeight, maxHeight);
    double midpoint = (boundedLower + boundedUpper) * 0.5;
    double amplitude = (boundedUpper - boundedLower) * 0.5;
    double omegaRadPerSec = 2.0 * Math.PI * Math.max(0.0, frequencyHz);
    double[] startTimeSeconds = new double[1];

    return run(() -> {
      double elapsedTimeSeconds = Timer.getFPGATimestamp() - startTimeSeconds[0];
      double targetPosition = midpoint;

      if (omegaRadPerSec > 0.0 && amplitude > 0.0) {
        targetPosition = midpoint + amplitude * Math.sin(omegaRadPerSec * elapsedTimeSeconds);
      }

      setPosition(targetPosition);
    }).beforeStarting(() -> startTimeSeconds[0] = Timer.getFPGATimestamp());
  }

  /**
   * Creates a command that oscillates the intake between 0.1 m and max height
   * at a default frequency.
   *
   * @return A command that continuously applies a sinusoidal setpoint
   */
  public Command oscillateIntake() {
    return oscillateIntake(0.1, getMaxHeightMeters(), 0.5);
  }

}
