package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Hood sub-component using TalonFXS with NEO550 motor.
 * Not a subsystem – owned and driven by {@link frc.robot.subsystems.Shooter}.
 */
@Logged(name = "Hood")
public class Hood {

    // Constants
    private final DCMotor dcMotor = DCMotor.getNeo550(1);
    private final int canID = 10;
    private final double gearRatio = 199.39113;
    private final double kP = 1200;
    private final double kI = 0;
    private final double kD = 34;
    private final double kS = 0.3896484375;
    private final double kV = 0;
    private final double kA = 0;
    private final double kG = 0;
    private final boolean brakeMode = true;
    private final boolean enableStatorLimit = true;
    private final double statorCurrentLimit = 20;

    // Motor controller
    private final TalonFXS motor;
    private final PositionVoltage positionRequest;
    private final VelocityVoltage velocityRequest;
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Temperature> temperatureSignal;

    // Simulation
    private final SingleJointedArmSim pivotSim;

    /**
     * Creates a new Hood Subsystem.
     */
    public Hood() {
        // Initialize motor controller
        motor = new TalonFXS(canID, new CANBus("*"));
        System.out.println("actually running1 ");

        // Create control requests
        positionRequest = new PositionVoltage(0).withSlot(0);
        velocityRequest = new VelocityVoltage(0).withSlot(0);

        // Get status signals
        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        voltageSignal = motor.getMotorVoltage();
        statorCurrentSignal = motor.getStatorCurrent();
        temperatureSignal = motor.getDeviceTemp();
        System.out.println("actually running2");
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        CommutationConfigs commutation = config.Commutation;
        commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;
        commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = .085;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        // Configure PID for slot 0
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;
        slot0.kS = kS;
        slot0.kV = kV;
        slot0.kA = kA;
        slot0.kG = kG;
        System.out.println("actually running 3");

        // Set current limits
        CurrentLimitsConfigs currentLimits = config.CurrentLimits;
        currentLimits.StatorCurrentLimit = statorCurrentLimit;
        currentLimits.StatorCurrentLimitEnable = enableStatorLimit;
                currentLimits.SupplyCurrentLimit = statorCurrentLimit;
        currentLimits.SupplyCurrentLimitEnable = enableStatorLimit;


        // Set brake mode
        config.MotorOutput.NeutralMode = brakeMode
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;

        // Configure gear ratio (sensor to mechanism)
        config.ExternalFeedback.withExternalFeedbackSensorSource(
                ExternalFeedbackSensorSourceValue.Commutation)
                .withSensorToMechanismRatio(gearRatio);
        System.out.println("actually running4");

        motor.getConfigurator().apply(config);

        motor.setPosition(0);
        System.out.println("actually running5");

        // Initialize simulation
        pivotSim = new SingleJointedArmSim(
                dcMotor, // Motor type
                gearRatio,
                0.005771941, // Arm moment of inertia
                Units.inchesToMeters(3.883), // Arm length (m)
                Units.degreesToRadians(-90), // Min angle (rad)
                Units.degreesToRadians(90), // Max angle (rad)
                false,
                Units.degreesToRadians(0) // Starting position (rad)
        );
    }

    /**
     * Update telemetry.
     */
    public void periodic() {
        BaseStatusSignal.refreshAll(
                positionSignal,
                velocitySignal
                );
    }

    /**
     * Update simulation.
     */
    public void simulationPeriodic() {
        // Set supply voltage for the TalonFXS sim state
        motor.getSimState().setSupplyVoltage(12);

        // Set input voltage from motor controller to simulation
        pivotSim.setInput(motor.getSimState().getMotorVoltage());

        // Update simulation by 20ms
        pivotSim.update(0.020);
        pivotSim.setInputVoltage(12);
        // Apply rotor position/velocity (before gear ratio) to sim state
        double motorPosition = Radians.of(pivotSim.getAngleRads() * gearRatio).in(Rotations);
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
        double positionRotations = Units.degreesToRotations(angleDegrees);
        motor.setControl(positionRequest.withPosition(positionRotations));
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

    public Pose3d getMechanismPose(Rotation3d turretRotation) {
        double angleRadians = Rotations.of(getPosition()).in(Radians);
        return new Pose3d(new Translation3d(0.2110268, 0.1016, 0.504698).rotateBy(new Rotation3d(0,0,Math.PI)),
                new Rotation3d(0, -0.659734457 + angleRadians, 0).plus(turretRotation));
    }

    public boolean isNearTarget(Angle threshold) {
        return positionSignal.isNear(positionRequest.getPositionMeasure(), threshold);
    }
}
