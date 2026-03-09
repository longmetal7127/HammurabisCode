package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.CRTSolver;
import frc.robot.util.CRTSolverConfig;

/**
 * Turret sub-component using TalonFX with Krakenx60 motor.
 * Not a subsystem – owned and driven by {@link frc.robot.subsystems.Shooter}.
 */
@Logged(name = "Turret")
public class Turret {

  // Constants
  private final DCMotor dcMotor = DCMotor.getKrakenX60(1);
  private final int canID = 11;

  private final int platterGearTeeth = 90;
  private final int encoder1Teeth = 19;
  private final int encoder2Teeth = 9;
  private final double gearRatio = 34.5;

  private final double kP = 90;
  private final double kI = 0;
  private final double kD = 1.2881;
  private final double kS = 0.24727;
  private final double kV = 4.1561;
  private final double kA = 0.0604;
  private final double maxVelocity = .5;
  private final double maxAcceleration = 1;
  private final boolean brakeMode = true;
  private final boolean enableStatorLimit = true;
  private final double statorCurrentLimit = 40;
  private final boolean enableSupplyLimit = false;
  private final double supplyCurrentLimit = 40;

  // Motor controller
  private final TalonFX motor = new TalonFX(canID, new CANBus("*"));
;
  private final PositionVoltage positionRequest;
  private final VelocityVoltage velocityRequest;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Temperature> temperatureSignal;

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

    // Create control requests
    positionRequest = new PositionVoltage(0).withSlot(0);
    velocityRequest = new VelocityVoltage(0).withSlot(0);

    // Get status signals
    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    temperatureSignal = motor.getDeviceTemp();

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Configure PID for slot 0
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;

    // Set current limits
    CurrentLimitsConfigs currentLimits = config.CurrentLimits;
    currentLimits.StatorCurrentLimit = statorCurrentLimit;
    currentLimits.StatorCurrentLimitEnable = enableStatorLimit;
    currentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    currentLimits.SupplyCurrentLimitEnable = enableSupplyLimit;

    // Set brake mode
    config.MotorOutput.NeutralMode = brakeMode
        ? NeutralModeValue.Brake
        : NeutralModeValue.Coast;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(145);
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(-145);
    config.Feedback.SensorToMechanismRatio = gearRatio;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    motor.getConfigurator().apply(config);

    motor.setPosition(0);

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

  public void periodic() {
    BaseStatusSignal.refreshAll(
        positionSignal,
        velocitySignal,
        voltageSignal,
        statorCurrentSignal,
        temperatureSignal);

    if (!positionInitialized && false) {
      Optional<Angle> solvedAngle = crtSolver.getAngleOptional();
      if (solvedAngle.isPresent()) {
        double angleDegrees = solvedAngle.get().in(Degrees);
        // Set motor position in rotations (mechanism rotations, since SensorToMechanismRatio is set)
        motor.setPosition(Units.degreesToRotations(angleDegrees));
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
  public void simulationPeriodic() {
    // Set supply voltage for the TalonFX sim state
    motor.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    // Set input voltage from motor controller to simulation
    pivotSim.setInput(motor.getSimState().getMotorVoltage());

    // Update simulation by 20ms
    pivotSim.update(0.020);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            pivotSim.getCurrentDrawAmps()));

    // Apply rotor position/velocity (before gear ratio) to sim state
    double motorPosition = Radians.of(pivotSim.getAngleRads() * gearRatio).in(
        Rotations);
    double motorVelocity = RadiansPerSecond.of(
        pivotSim.getVelocityRadPerSec() * gearRatio).in(RotationsPerSecond);

    motor.getSimState().setRawRotorPosition(motorPosition);
    motor.getSimState().setRotorVelocity(motorVelocity);
  }

  /**
   * Get the current position in Rotations.
   * 
   * @return Position in Rotations
   */
  @Logged(name = "Position/Rotations")
  public double getPosition() {
    // Rotations (mechanism rotations, since SensorToMechanismRatio is set)
    return positionSignal.getValueAsDouble();
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
    return velocitySignal.getValueAsDouble();
  }

  /**
   * Get the current applied voltage.
   * 
   * @return Applied voltage
   */
  @Logged(name = "Voltage")
  public double getVoltage() {
    return voltageSignal.getValueAsDouble();
  }

  /**
   * Get the current motor current.
   * 
   * @return Motor current in amps
   */
  public double getCurrent() {
    return statorCurrentSignal.getValueAsDouble();
  }

  /**
   * Get the current motor temperature.
   * 
   * @return Motor temperature in Celsius
   */
  public double getTemperature() {
    return temperatureSignal.getValueAsDouble();
  }

  /**
   * Set pivot angle.
   * 
   * @param angleDegrees The target angle in degrees
   */
  public void setAngle(double angleDegrees) {
    // Convert degrees to rotations
    double positionRotations = Units.degreesToRotations(angleDegrees);
    setpoint = angleDegrees;
    motor.setControl(positionRequest.withPosition(positionRotations));
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

    motor.setControl(velocityRequest.withVelocity(velocityRotations));
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
