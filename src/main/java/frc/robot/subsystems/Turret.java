package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;
import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.CRTSolver;
import frc.robot.util.CRTSolverConfig;

/**
 * Pivot subsystem using SparkFlex with NEO motor
 */
@Logged(name = "Turret")
public class Turret extends SubsystemBase {

  // Constants
  private final DCMotor dcMotor = DCMotor.getNEO(1);
  private final int canID = 20;

  private final int platterGearTeeth = 90;
  private final int encoder1Teeth = 19;
  private final int encoder2Teeth = 9;
  private final double gearRatio = 34.5;

  private final double kP = 1.84;
  private final double kI = 0;
  private final double kD = 0;
  private final double kS = 0;
  private final double kV = 3.64;
  private final double kA = 0.09;
  private final double maxVelocity = .5;
  private final double maxAcceleration = 1;
  private final boolean brakeMode = true;


  // Motor controller
  private final SparkFlex motor;
  private final RelativeEncoder encoder;
  private final SparkSim motorSim;
  private final SparkClosedLoopController sparkPidController;

  private final double minAngleDegrees = -180.0;
  private final double maxAngleDegrees = 180.0;

  private final int encoder1Port = 0;
  private final int encoder2Port = 1;

  // Simulation
  private final SingleJointedArmSim pivotSim;

  private final CRTSolver crtSolver;
  private boolean positionInitialized = false;
  private final DutyCycleEncoder absoluteEncoder1;
  private final DutyCycleEncoder absoluteEncoder2;
  private final DutyCycleEncoderSim absoluteEncoder1Sim;
  private final DutyCycleEncoderSim absoluteEncoder2Sim;

  public double setpoint = 0.0;

  /**
   * Creates a new Pivot Subsystem.
   */
  public Turret() {
    // Initialize motor controller
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motor = new SparkFlex(canID, MotorType.kBrushless);
    motorConfig.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

    // Configure encoder
    encoder = motor.getEncoder();
    encoder.setPosition(0);

    // Set current limits
    motorConfig.smartCurrentLimit(40);

    // Configure Feedback and Feedforward
    sparkPidController = motor.getClosedLoopController();
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kP, kI, kD, ClosedLoopSlot.kSlot0);
    motorConfig.closedLoop.feedForward.kS(kS).kV(kV).kA(kA);
    motorConfig.closedLoop.feedForward.kG(0);
    motorConfig.closedLoop.maxMotion
        .cruiseVelocity(maxVelocity)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .allowedProfileError(0.02)
        .maxAcceleration(maxAcceleration);

    // Configure Encoder Gear Ratio
    motorConfig.encoder
        .positionConversionFactor(1 / gearRatio)
        .velocityConversionFactor((1 / gearRatio) / 60); // Covnert RPM to RPS

    // Save configuration
    motor.configure(
        motorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    motorSim = new SparkSim(motor, dcMotor);

    // Initialize simulation
    pivotSim = new SingleJointedArmSim(
        dcMotor, // Motor type
        gearRatio,
        0.01, // Arm moment of inertia - Small value since there are no arm parameters
        0.1, // Arm length (m) - Small value since there are no arm parameters
        Units.degreesToRadians(-90), // Min angle (rad)
        Units.degreesToRadians(90), // Max angle (rad)
        false, // Simulate gravity - Disable gravity for pivot
        Units.degreesToRadians(0) // Starting position (rad)
    );

    absoluteEncoder1 = new DutyCycleEncoder(encoder1Port);
    absoluteEncoder2 = new DutyCycleEncoder(encoder2Port);
    absoluteEncoder1Sim = new DutyCycleEncoderSim(absoluteEncoder1);
    absoluteEncoder2Sim = new DutyCycleEncoderSim(absoluteEncoder2);

    CRTSolverConfig crtConfig = new CRTSolverConfig(
        () -> Rotations.of(absoluteEncoder1.get()),
        () -> Rotations.of(absoluteEncoder2.get()));

    crtConfig
        .withCommonDriveGear(
            34.5,
            platterGearTeeth,
            encoder1Teeth,
            encoder2Teeth)
        .withMechanismRange(
            Degrees.of(minAngleDegrees),
            Degrees.of(maxAngleDegrees))
        .withMatchTolerance(Rotations.of(0.03));

    crtSolver = new CRTSolver(crtConfig);

  }

  /**
   * Update simulation and telemetry.
   */

  @Override
  public void periodic() {
    if (!positionInitialized && !Robot.isSimulation()) {
      Optional<Angle> solvedAngle = crtSolver.getAngleOptional();
      if (solvedAngle.isPresent()) {
        double angleDegrees = solvedAngle.get().in(Degrees);
        encoder.setPosition(angleDegrees);
        positionInitialized = true;
        System.out.println("Turret position initialized to: " + angleDegrees + " degrees");
        System.out.println("CRT Status: " + crtSolver.getLastStatus());
        System.out.println("CRT Error: " + crtSolver.getLastErrorRotations() + " rotations");
      } else {
        if (crtSolver.getLastStatus() != CRTSolver.CRTStatus.NOT_ATTEMPTED) {
          System.out.println("CRT initialization failed: " + crtSolver.getLastStatus());
          System.out.println("CRT Error: " + crtSolver.getLastErrorRotations() + " rotations");
        }
      }
    }
  }

  /**
   * Update simulation.
   */
  @Override
  public void simulationPeriodic() {
    // Set input voltage from motor controller to simulation
    // Note: This may need to be talonfx.getSimState().getMotorVoltage() as the
    // input
    // pivotSim.setInput(dcMotor.getVoltage(dcMotor.getTorque(pivotSim.getCurrentDrawAmps()),
    // pivotSim.getVelocityRadPerSec()));
    // pivotSim.setInput(getVoltage());
    // Set input voltage from motor controller to simulation
    // Use getVoltage() for other controllers
    pivotSim.setInput(getVoltage());

    // Update simulation by 20ms
    pivotSim.update(0.020);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            pivotSim.getCurrentDrawAmps()));

    double motorPosition = Radians.of(pivotSim.getAngleRads() * gearRatio).in(
        Rotations);
    double motorVelocity = RadiansPerSecond.of(
        pivotSim.getVelocityRadPerSec()).in(RotationsPerSecond);
    motorSim.iterate(motorVelocity, RoboRioSim.getVInVoltage(), 0.02);
  }

  /**
   * Get the current position in Rotations.
   * 
   * @return Position in Rotations
   */
  @Logged(name = "Position/Rotations")
  public double getPosition() {
    // Rotations
    return encoder.getPosition();
  }

  public double getPositionRadians() {
    return Units.rotationsToRadians(getPosition());
  }

  public double getAngleDegrees() {
    return Units.radiansToDegrees(getPositionRadians());
  }

  /**
   * Get the current velocity in rotations per second.
   * 
   * @return Velocity in rotations per second
   */
  @Logged(name = "Velocity")
  public double getVelocity() {
    return encoder.getVelocity() / gearRatio / 60.0; // Convert from RPM to RPS
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
   * Set pivot angle with acceleration.
   * 
   * @param angleDegrees The target angle in degrees
   */
  public void setAngle(double angleDegrees) {
    // Convert degrees to rotations
    double angleRotations = Units.degreesToRotations(angleDegrees);
    setpoint = angleDegrees;
    sparkPidController.setSetpoint(
        angleRotations,
        ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0);
  }

  /**
   * Set pivot angular velocity.
   * 
   * @param velocityDegPerSec The target velocity in degrees per second
   */
  public void setVelocity(double velocityDegPerSec) {
    setVelocity(velocityDegPerSec, 0);
  }

  /**
   * Set pivot angular velocity with acceleration.
   * 
   * @param velocityDegPerSec The target velocity in degrees per second
   * @param acceleration      The acceleration in degrees per second squared
   */
  public void setVelocity(double velocityDegPerSec, double acceleration) {
    // Convert degrees/sec to rotations/sec
    double velocityRadPerSec = Units.degreesToRadians(velocityDegPerSec);
    double velocityRotations = velocityRadPerSec / (2.0 * Math.PI);

    sparkPidController.setSetpoint(
        velocityRotations,
        ControlType.kVelocity,
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
   * Get the pivot simulation for testing.
   * 
   * @return The pivot simulation model
   */
  public SingleJointedArmSim getSimulation() {
    return pivotSim;
  }

  /**
   * Creates a command to set the pivot to a specific angle.
   * 
   * @param angleDegrees The target angle in degrees
   * @return A command that sets the pivot to the specified angle
   */
  public Command setAngleCommand(double angleDegrees) {
    return runOnce(() -> setAngle(angleDegrees));
  }

  public Command followAngleCommand(Supplier<Double> angleDegreesSupplier) {
    return run(() -> setAngle(angleDegreesSupplier.get()));
  } 


  /**
   * Creates a command to stop the pivot.
   * 
   * @return A command that stops the pivot
   */
  public Command stopCommand() {
    return runOnce(() -> setVelocity(0));
  }

  /**
   * Creates a command to move the pivot at a specific velocity.
   * 
   * @param velocityDegPerSec The target velocity in degrees per second
   * @return A command that moves the pivot at the specified velocity
   */
  public Command moveAtVelocityCommand(double velocityDegPerSec) {
    return run(() -> setVelocity(velocityDegPerSec));
  }

  /**
   * Gets the 3D pose of the turret mechanism for visualization.
   * The turret rotates around the Z axis (yaw).
   * 
   * @return Pose3d representing the turret's position and orientation
   */
  public Pose3d getMechanismPose() {
    return new Pose3d(
        new Translation3d(0.1651, 0.1016, 0.3349752),
        new Rotation3d(0, 0, Math.toRadians(getAngleDegrees())));
  }

}
