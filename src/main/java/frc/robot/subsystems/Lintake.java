package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TiltedElevatorSim;

/**
 * Elevator subsystem using SparkFlex with NEO motor
 */
@Logged(name = "Lintake")
public class Lintake extends SubsystemBase {

  // Constants
  private final DCMotor dcMotor = DCMotor.getNeoVortex(1);
  private final int canID = 1;
  // TODO: Gear ratio is not populated
  private final double gearRatio = 15;
  private final double kP = 1;
  private final double kI = 0;
  private final double kD = 0;
  private final double kS = 0;
  private final double kV = 0;
  private final double kA = 0;
  private final double kG = 0;
  private final double maxVelocity = 1; // meters per second
  private final boolean brakeMode = true;
  private final int statorCurrentLimit = 40;
  private final double drumRadius = 0.0254; // meters
  private final double minheight = 0;
  private final double maxheight = 1;

  // Feedforward


  // Motor controller
  private final SparkFlex motor;
  private final RelativeEncoder encoder;
  private final SparkSim motorSim;
  private final SparkClosedLoopController sparkPidController;

  // Simulation
  private final TiltedElevatorSim elevatorSim;

  /**
   * Creates a new Elevator Subsystem.
   */
  public Lintake() {
    // Initialize motor controller
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motor = new SparkFlex(canID, MotorType.kBrushless);
    motorConfig.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

    // Configure encoder
    encoder = motor.getEncoder();
    encoder.setPosition(0);

    // Set ramp rates

    //Set current limits
    motorConfig.smartCurrentLimit(statorCurrentLimit);

    // Configure Feedback and Feedforward
    sparkPidController = motor.getClosedLoopController();
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(kP, kI, kD, ClosedLoopSlot.kSlot0);
    motorConfig.closedLoop.feedForward.kS(kS).kV(kV).kA(kA);
    motorConfig.closedLoop.feedForward.kG(kG);

    // Configure Encoder Gear Ratio
    motorConfig.encoder
      .positionConversionFactor(1 / gearRatio)
      .velocityConversionFactor((1 / gearRatio) / 60); // Covnert RPM to RPS

    // Save configuration
    motor.configure(
      motorConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    motorSim = new SparkSim(motor, dcMotor);

    // Initialize simulation
    elevatorSim = new TiltedElevatorSim(
      dcMotor, // Motor type
      gearRatio,
      Units.lbsToKilograms(12.58), // Carriage mass (kg)
      drumRadius, // Drum radius (m)
      0, // Min height (m)
      1, // Max height (m)
      true, // Simulate gravity
      0 // Starting height (m)
    );
  }

  /**
   * Update simulation and telemetry.
   */
  @Override
  public void periodic() {}

  /**
   * Update simulation.
   */
  @Override
  public void simulationPeriodic() {
    // Meters to Rotations Ratio
    double positionToRotations = (1 / (2.0 * Math.PI * drumRadius)) * gearRatio;

    // Set input voltage from motor controller to simulation
    // Note: This may need to be talonfx.getSimState().getMotorVoltage() as the input
    //elevatorSim.setInput(dcMotor.getVoltage(dcMotor.getTorque(elevatorSim.getCurrentDrawAmps()), elevatorSim.getVelocityMetersPerSecond() * positionToRotations * 2 * Math.PI));
    // elevatorSim.setInput(getVoltage());

    // Use getVoltage() for other controllers
    elevatorSim.setInput(getVoltage());

    // Update simulation by 20ms
    elevatorSim.update(0.020);

    // Convert meters to motor rotations
    double motorPosition =
      elevatorSim.getPositionMeters() * positionToRotations;
    double motorVelocity =
      elevatorSim.getVelocityMetersPerSecond() * positionToRotations;

    motorSim.iterate(motorVelocity * 60, RoboRioSim.getVInVoltage(), 0.02);
  }

  /**
   * Get the current position in the Rotations.
   * @return Position in Rotations
   */
  @Logged(name = "Position/Rotations")
  public double getPosition() {
    // Rotations
    return encoder.getPosition() / gearRatio;
  }

  /**
   * Get the current velocity in rotations per second.
   * @return Velocity in rotations per second
   */
  @Logged(name = "Velocity")
  public double getVelocity() {
    return encoder.getVelocity() / gearRatio / 60.0; // Convert from RPM to RPS
  }

  /**
   * Get the current applied voltage.
   * @return Applied voltage
   */
  @Logged(name = "Voltage")
  public double getVoltage() {
    return motor.getAppliedOutput() * motor.getBusVoltage();
  }

  /**
   * Get the current motor current.
   * @return Motor current in amps
   */
  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  /**
   * Get the current motor temperature.
   * @return Motor temperature in Celsius
   */
  public double getTemperature() {
    return motor.getMotorTemperature();
  }

  /**
   * Set elevator position.
   * @param position The target position in meters
   */
  public void setPosition(double position) {
    setPosition(position, 0);
  }

  /**
   * Set elevator position with acceleration.
   * @param position The target position in meters
   * @param acceleration The acceleration in meters per second squared
   */
  public void setPosition(double position, double acceleration) {
    // Convert meters to rotations
    double positionRotations = position / (2.0 * Math.PI * drumRadius);

    sparkPidController.setSetpoint(
      positionRotations,
      ControlType.kMAXMotionPositionControl,
      ClosedLoopSlot.kSlot0
    );
  }

  /**
   * Set elevator velocity.
   * @param velocity The target velocity in meters per second
   */
  public void setVelocity(double velocity) {
    setVelocity(velocity, 0);
  }

  /**
   * Set elevator velocity with acceleration.
   * @param velocity The target velocity in meters per second
   * @param acceleration The acceleration in meters per second squared
   */
  public void setVelocity(double velocity, double acceleration) {
    // Convert meters/sec to rotations/sec
    double velocityRotations = velocity / (2.0 * Math.PI * drumRadius);

    sparkPidController.setSetpoint(
      velocityRotations,
      ControlType.kVelocity,
      ClosedLoopSlot.kSlot0
    );
  }

  /**
   * Set motor voltage directly.
   * @param voltage The voltage to apply
   */
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /**
   * Get the elevator simulation for testing.
   * @return The elevator simulation model
   */
  public TiltedElevatorSim getSimulation() {
    return elevatorSim;
  }

  public double getMinHeightMeters() {
    return minheight;
  }

  public double getMaxHeightMeters() {
    return maxheight;
  }

  /**
   * Creates a command to set the elevator to a specific height.
   * @param heightMeters The target height in meters
   * @return A command that sets the elevator to the specified height
   */
  public Command setHeightCommand(double heightMeters) {
    return runOnce(() -> setPosition(heightMeters));
  }

  /**
   * Creates a command to move the elevator to a specific height with a profile.
   * @param heightMeters The target height in meters
   * @return A command that moves the elevator to the specified height
   */
  public Command moveToHeightCommand(double heightMeters) {
    return run(() -> {
      double currentHeight = getPosition() * (2.0 * Math.PI * drumRadius);
      double error = heightMeters - currentHeight;
      double velocity =
        Math.signum(error) * Math.min(Math.abs(error) * 2.0, maxVelocity);
      setVelocity(velocity);
    }).until(() -> {
      double currentHeight = getPosition() * (2.0 * Math.PI * drumRadius);
      return Math.abs(heightMeters - currentHeight) < 0.02; // 2cm tolerance
    });
  }

  /**
   * Creates a command to stop the elevator.
   * @return A command that stops the elevator
   */
  public Command stopCommand() {
    return runOnce(() -> setVelocity(0));
  }

  /**
   * Creates a command to move the elevator at a specific velocity.
   * @param velocityMetersPerSecond The target velocity in meters per second
   * @return A command that moves the elevator at the specified velocity
   */
  public Command moveAtVelocityCommand(double velocityMetersPerSecond) {
    return run(() -> setVelocity(velocityMetersPerSecond));
  }
}
