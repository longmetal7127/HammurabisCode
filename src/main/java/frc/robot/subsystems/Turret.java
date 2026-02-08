package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CRTSolver;
import frc.robot.util.CRTSolverConfig;
import java.util.Optional;

/**
 * Turret subsystem using SparkFlex with NEO Vortex motor. Position is initialized using CRTSolver
 * with two REV through bore encoders.
 */
@Logged(name = "Turret")
public class Turret extends SubsystemBase {

  private final DCMotor dcMotor = DCMotor.getNeoVortex(1);
  private final int canID = 20;
  
  private final int platterGearTeeth = 90;
  private final int encoder1Teeth = 19;
  private final int encoder2Teeth = 9;
  private final double gearRatio = 34.5; 
  
  private final double kP = 0.5;
  private final double kI = 0.0;
  private final double kD = 0.0;
  private final double kS = 0.0;
  private final double kV = 0.0;
  private final double kA = 0.0;
  
  private final int statorCurrentLimit = 40;
  
  private final double minAngleDegrees = -180.0;
  private final double maxAngleDegrees = 180.0;
  

  private final int encoder1Port = 0; 
  private final int encoder2Port = 1; 


  private final SparkFlex motor;
  private final RelativeEncoder encoder;
  private final SparkSim motorSim;
  private final SparkClosedLoopController sparkPidController;


  private final DutyCycleEncoder absoluteEncoder1;
  private final DutyCycleEncoder absoluteEncoder2;
  private final DutyCycleEncoderSim absoluteEncoder1Sim;
  private final DutyCycleEncoderSim absoluteEncoder2Sim;
  
  private final CRTSolver crtSolver;
  private boolean positionInitialized = false;

  private final LinearSystem<N2, N1, N2> turretSystem;
  private final LinearSystemSim<N2, N1, N2> turretSim;

  public Turret() {
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motor = new SparkFlex(canID, MotorType.kBrushless);
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.smartCurrentLimit(statorCurrentLimit);

    encoder = motor.getEncoder();
    sparkPidController = motor.getClosedLoopController();
    
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kP, kI, kD, ClosedLoopSlot.kSlot0);
    motorConfig.closedLoop.feedForward.kS(kS).kV(kV).kA(kA);

    motorConfig.encoder
        .positionConversionFactor(360.0 / gearRatio)
        .velocityConversionFactor(360.0 / gearRatio / 60.0); 

    motorConfig.softLimit
        .forwardSoftLimit(maxAngleDegrees)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(minAngleDegrees)
        .reverseSoftLimitEnabled(true);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorSim = new SparkSim(motor, dcMotor);

    absoluteEncoder1 = new DutyCycleEncoder(encoder1Port);
    absoluteEncoder2 = new DutyCycleEncoder(encoder2Port);
    absoluteEncoder1Sim = new DutyCycleEncoderSim(absoluteEncoder1);
    absoluteEncoder2Sim = new DutyCycleEncoderSim(absoluteEncoder2);

    CRTSolverConfig crtConfig = new CRTSolverConfig(
        () -> Rotations.of(absoluteEncoder1.get()),
        () -> Rotations.of(absoluteEncoder2.get())
    );
    
    crtConfig
        .withCommonDriveGear(
            34.5,
            platterGearTeeth,
            encoder1Teeth,       
            encoder2Teeth
        )
        .withMechanismRange(
            Degrees.of(minAngleDegrees),
            Degrees.of(maxAngleDegrees)
        )
        .withMatchTolerance(Rotations.of(0.03)); 
    
    crtSolver = new CRTSolver(crtConfig);

    turretSystem = LinearSystemId.createDCMotorSystem(dcMotor, 0.095624, gearRatio);
    turretSim = new LinearSystemSim<>(turretSystem);
  }

  @Override
  public void periodic() {
    if (!positionInitialized) {
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

  @Override
  public void simulationPeriodic() {
    double voltage = motor.getAppliedOutput() * motor.getBusVoltage();
    
    turretSim.setInput(voltage);
    turretSim.update(0.02);
    
    double turretAngleRad = turretSim.getOutput(0); 
    double turretVelocityRadPerSec = turretSim.getOutput(1);
    
    double turretAngleRotations = turretAngleRad / (2.0 * Math.PI);
    
    double encoder1Reading = turretAngleRotations * platterGearTeeth / encoder1Teeth;
    double encoder2Reading = turretAngleRotations * platterGearTeeth / encoder2Teeth;
    
    encoder1Reading = encoder1Reading - Math.floor(encoder1Reading);
    encoder2Reading = encoder2Reading - Math.floor(encoder2Reading);
    
    absoluteEncoder1Sim.set(encoder1Reading);
    absoluteEncoder2Sim.set(encoder2Reading);
    
    double turretVelocityDegreesPerSec = Math.toDegrees(turretVelocityRadPerSec);
    motorSim.iterate(turretVelocityDegreesPerSec * 60 / 360, RobotController.getBatteryVoltage(), 0.02);
  }

  /**
   * Sets the turret angle setpoint in degrees.
   * Clamps to [-180, 180] range for wire safety.
   * 
   * @param angleDegrees target angle in degrees
   */
  public void setAngle(double angleDegrees) {
    if (positionInitialized) {
      double clampedAngle = Math.max(minAngleDegrees, Math.min(maxAngleDegrees, angleDegrees));
      sparkPidController.setSetpoint(clampedAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
  }


  /**
   * Sets motor voltage directly.
   * 
   * @param voltage voltage to apply
   */
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /**
   * Gets the current turret angle in degrees.
   * 
   * @return current angle in degrees
   */
  public double getAngleDegrees() {
    return encoder.getPosition();
  }

  /**
   * Gets the current turret velocity in degrees per second.
   * 
   * @return velocity in degrees per second
   */
  public double getVelocityDegreesPerSec() {
    return encoder.getVelocity();
  }

  /**
   * Gets the current applied voltage.
   * 
   * @return applied voltage
   */
  public double getVoltage() {
    return motor.getAppliedOutput() * motor.getBusVoltage();
  }

  /**
   * Gets the current motor current in amps.
   * 
   * @return motor current in amps
   */
  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  /**
   * Gets the current motor temperature in Celsius.
   * 
   * @return motor temperature in Celsius
   */
  public double getTemperature() {
    return motor.getMotorTemperature();
  }

  /**
   * Returns whether the turret position has been initialized.
   * 
   * @return true if position is initialized
   */
  public boolean isPositionInitialized() {
    return positionInitialized;
  }

  /**
   * Gets the CRT solver status.
   * 
   * @return last CRT solver status
   */
  public CRTSolver.CRTStatus getCRTStatus() {
    return crtSolver.getLastStatus();
  }

  /**
   * Gets the minimum angle limit in degrees.
   * 
   * @return minimum angle in degrees
   */
  public double getMinAngleDegrees() {
    return minAngleDegrees;
  }

  /**
   * Gets the maximum angle limit in degrees.
   * 
   * @return maximum angle in degrees
   */
  public double getMaxAngleDegrees() {
    return maxAngleDegrees;
  }

  /**
   * Checks if turret is within tolerance of target angle.
   * 
   * @param targetAngleDegrees target angle in degrees
   * @param toleranceDegrees tolerance in degrees
   * @return true if within tolerance
   */
  public boolean isAtAngle(double targetAngleDegrees, double toleranceDegrees) {
    return Math.abs(getAngleDegrees() - targetAngleDegrees) < toleranceDegrees;
  }

  /**
   * Stops the turret motor.
   */
  public void stop() {
    motor.stopMotor();
  }

  /**
   * Gets the turret simulation for testing.
   * 
   * @return turret simulation model
   */
  public LinearSystemSim<N2, N1, N2> getSimulation() {
    return turretSim;
  }

  /**
   * Command to set turret to a specific angle.
   * 
   * @param angleDegrees target angle in degrees
   * @return command that sets the angle
   */
  public Command setAngleCommand(double angleDegrees) {
    return runOnce(() -> setAngle(angleDegrees))
        .withName("Turret.SetAngle(" + angleDegrees + ")");
  }

  /**
   * Command to move turret to a specific angle using a supplier.
   * 
   * @param angleSupplier supplier that returns target angle in degrees
   * @return command that continuously updates the setpoint
   */
  public Command trackAngleCommand(java.util.function.Supplier<Double> angleSupplier) {
    return run(() -> setAngle(angleSupplier.get()))
        .withName("Turret.TrackAngle");
  }

  /**
   * Command to move turret to angle and wait until it reaches the target.
   * 
   * @param angleDegrees target angle in degrees
   * @param toleranceDegrees tolerance in degrees (default 2.0)
   * @return command that finishes when at target
   */
  public Command moveToAngleCommand(double angleDegrees, double toleranceDegrees) {
    return run(() -> setAngle(angleDegrees))
        .until(() -> isAtAngle(angleDegrees, toleranceDegrees))
        .withName("Turret.MoveToAngle(" + angleDegrees + ")");
  }

  /**
   * Command to move turret to angle and wait until it reaches the target (2 degree tolerance).
   * 
   * @param angleDegrees target angle in degrees
   * @return command that finishes when at target
   */
  public Command moveToAngleCommand(double angleDegrees) {
    return moveToAngleCommand(angleDegrees, 2.0);
  }

  /**
   * Command to stop the turret.
   * 
   * @return command to run
   */
  public Command stopCommand() {
    return runOnce(this::stop)
        .withName("Turret.Stop");
  }

  /**
   * Command to home/zero the turret to center position (0 degrees).
   * 
   * @return command to run
   */
  public Command homeCommand() {
    return moveToAngleCommand(0.0)
        .withName("Turret.Home");
  }


}
