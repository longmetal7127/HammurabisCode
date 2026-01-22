package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

public class Shooter implements Subsystem {

    private TalonFX primaryMotor = new TalonFX(41, "main_canivore");
    private TalonFX secondaryMotor = new TalonFX(42, "main_canivore");

    private double sensorToMechanismRatio = 1.0 / 2.0;
    private Follower secondaryControl = new Follower(41, MotorAlignmentValue.Aligned);
    private double statorCurrentLimit = 80.0;
    private double supplyCurrentLimit = 80.0;
    private TalonFXConfiguration primaryConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(
                    new CurrentLimitsConfigs()
                            .withStatorCurrentLimit(statorCurrentLimit)
                            .withSupplyCurrentLimit(supplyCurrentLimit))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(sensorToMechanismRatio))
            .withSlot0(
                    new Slot0Configs()
                            .withKP(4)
                            .withKS(3));

    private TalonFXConfiguration secondaryConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(
                    statorCurrentLimit).withSupplyCurrentLimit(supplyCurrentLimit));
    public Trigger atSpeed = new Trigger(() -> primaryMotor.getClosedLoopError().isNear(0, 0.05));
    private final DCMotorSim m_motorSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(2), 0.001, sensorToMechanismRatio

            ),
            DCMotor.getKrakenX60Foc(2));

    public Shooter() {
        primaryMotor.getConfigurator().apply(primaryConfiguration);
        secondaryMotor.getConfigurator().apply(secondaryConfiguration);
        if (Robot.isSimulation()) {
            simulationInit();
        }
    }

    public Command setVelocity(double velocity) {
        return runOnce(() -> {
            VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(velocity);
            primaryMotor.setControl(velocityControl);
            secondaryMotor.setControl(secondaryControl);
        });
    }

    public void simulationInit() {
        var talonFXSim = primaryMotor.getSimState();
        talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    public void simulationPeriodic() {
        var talonFXSim = primaryMotor.getSimState();

        // set the supply voltage of the TalonFX
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
        m_motorSimModel.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(sensorToMechanismRatio));
        talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(sensorToMechanismRatio));
    }
}
