package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;

import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Flywheel.FlywheelSetpoint;
import frc.robot.util.FuelSim;
import frc.robot.util.ShootOnTheFlyCalculator;

/**
 * Shooter subsystem that orchestrates the turret, hood, and flywheel
 * sub-components. All shooting-related commands live here so the
 * command scheduler treats the entire mechanism as a single subsystem.
 */
@Logged
public class Shooter extends SubsystemBase {

    // ── Sub-components (not subsystems) ──────────────────────────────────
    @Logged
    public final Turret turret = new Turret();
    @Logged
    public final Hood hood = new Hood();
    @Logged
    public final Flywheel flywheel = new Flywheel();

    public enum ShooterMode {
            /** Stop and Shoot Against Hub */
            AgainstHub,
            /** Pass across the field */
            Pass,
            /** Autoaim turret */
            Autoaim
    }

    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier;
    private final FuelSim fuelSim;

    private final ShootOnTheFlyCalculator sotfCalculator = buildSOTFCalculator();

    private final DoubleSubscriber hoodAngleOffsetDeg = DogLog.tunable("Shooter/SOTF/HoodAngleOffsetDeg", 0.0,
            edu.wpi.first.units.Units.Degrees);
    private final DoubleSubscriber rpmOffset = DogLog.tunable("Shooter/SOTF/RpmOffset", 0.0,
            edu.wpi.first.units.Units.RPM);

    // ── Fuel-sim shooting ────────────────────────────────────────────────
    /** Minimum interval between simulated fuel spawns (seconds). */
    private static final double FUEL_SPAWN_INTERVAL = 0.25;
    /**
     * Flywheel radius used to convert angular velocity to linear exit speed
     * (meters).
     */
    private static final double FLYWHEEL_RADIUS = Units.inchesToMeters(2.0);

    private final Timer fuelSpawnTimer = new Timer();

    /**
     * @param poseSupplier                Supplies the robot's current field pose
     * @param fieldRelativeSpeedsSupplier Supplies the robot's field-relative
     *                                    chassis speeds
     * @param fuelSim                     The FuelSim instance for spawning
     *                                    simulated projectiles
     */
    public Shooter(
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier,
            FuelSim fuelSim) {
        this.poseSupplier = poseSupplier;
        this.fieldRelativeSpeedsSupplier = fieldRelativeSpeedsSupplier;
        this.fuelSim = fuelSim;
    }

    @Override
    public void periodic() {
        turret.periodic();
        hood.periodic();
        flywheel.periodic();
    }

    @Override
    public void simulationPeriodic() {
        turret.simulationPeriodic();
        hood.simulationPeriodic();
    }

    /**
     * Returns true when the robot is inside its own alliance zone
     * (X &lt; the alliance-zone vertical line).
     */
    private boolean isInAllianceZone() {
        double robotX = poseSupplier.get().getX();
        return robotX < FieldConstants.LinesVertical.allianceZone;
    }

    /**
     * Finds the nearest hub corner (on our alliance side) to the robot's current
     * position.
     */
    private Translation2d getClosestHubCorner() {
        Translation2d robot = poseSupplier.get().getTranslation();
        Translation2d[] corners = {
                FieldConstants.Hub.nearLeftCorner,
                FieldConstants.Hub.nearRightCorner,
                FieldConstants.Hub.farLeftCorner,
                FieldConstants.Hub.farRightCorner
        };

        Translation2d closest = corners[0];
        double minDist = robot.getDistance(closest);
        for (int i = 1; i < corners.length; i++) {
            double d = robot.getDistance(corners[i]);
            if (d < minDist) {
                minDist = d;
                closest = corners[i];
            }
        }
        return closest;
    }

    /**
     * Finds the nearest alliance-zone corner to the robot's current position.
     * Used when shooting from the neutral zone back toward our alliance zone.
     */
    private Translation2d getClosestAllianceZoneCorner() {
        Translation2d robot = poseSupplier.get().getTranslation();
        Translation2d[] corners = {
                FieldConstants.AllianceZoneCorners.left,
                FieldConstants.AllianceZoneCorners.right
        };

        return robot.getDistance(corners[0]) < robot.getDistance(corners[1])
                ? corners[0]
                : corners[1];
    }

    /**
     * Returns the shoot-on-the-fly result for the current robot state.
     * In alliance zone the target is the closest hub corner;
     * outside the alliance zone the target is the closest alliance zone corner.
     */
    private ShootOnTheFlyCalculator.ShooterCommand getCurrentSOTFCommand() {
        Pose2d robotPose = poseSupplier.get();
        Translation2d turretOffset = turret.getRobotRelativeTranslation().rotateBy(robotPose.getRotation());
        Translation2d turretPosition = robotPose.getTranslation().plus(turretOffset);
        Translation2d target = isInAllianceZone()
                ? getClosestHubCorner()
                : getClosestAllianceZoneCorner();
        ChassisSpeeds speeds = fieldRelativeSpeedsSupplier.get();
    ShootOnTheFlyCalculator.ShooterCommand cmd = sotfCalculator.calculate(
                turretPosition,
                new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                target,
                0.0);
    DogLog.log("Shooter/SOTF/EffectiveDistanceMeters", cmd.effectiveDistance());
    return cmd;
    }

    /**
     * Builds the shoot-on-the-fly calculator with placeholder table entries.
     * TODO: fill with real characterisation data.
     */
    private static ShootOnTheFlyCalculator buildSOTFCalculator() {
        ShootOnTheFlyCalculator calc = new ShootOnTheFlyCalculator();
        // distance (m) → RPM, hood angle (deg), time-of-flight (s)
        calc.addTableEntry(1.0, 2000, 5, 0.3);
        calc.addTableEntry(2.0, 2800, 15, 0.45);
        calc.addTableEntry(3.0, 3400, 25, 0.6);
        calc.addTableEntry(4.0, 3900, 35, 0.75);
        calc.addTableEntry(5.0, 4300, 42, 0.9);
        calc.addTableEntry(6.0, 4600, 48, 1.05);
        return calc;
    }

    // ── Fuel simulation ──────────────────────────────────────────────────

    /**
     * Spawns a simulated fuel projectile into the FuelSim using
     * {@link FuelSim#launchFuel} based on the current turret angle,
     * hood angle, and flywheel speed.
     *
     * <p>
     * Only spawns in simulation and rate-limited by {@link #FUEL_SPAWN_INTERVAL}.
     */
    private void trySpawnSimulatedFuel() {
        if (!RobotBase.isSimulation())
            return;
        if (fuelSpawnTimer.get() < FUEL_SPAWN_INTERVAL)
            return;
        fuelSpawnTimer.restart();

        double hoodPitchRad = Units.rotationsToRadians(hood.getPosition());

        double turretYawRad = Math.toRadians(turret.getAngleDegrees());

        double flywheelRPS = flywheel.getleaderMotorVelocity().in(RotationsPerSecond);
        double exitSpeedMps = Math.abs(flywheelRPS) * 2.0 * Math.PI * FLYWHEEL_RADIUS;

        fuelSim.launchFuel(
                MetersPerSecond.of(exitSpeedMps),
                Radians.of(hoodPitchRad),
                Radians.of(turretYawRad),
                Inches.of(22)); // launch height — above bumper height
    }

    // ── Commands ─────────────────────────────────────────────────────────

    /**
     * Builds a command that continuously aims and shoots while held.
     *
     * <p>
     * The SOTF calculator is always used for turret angle, hood angle, and
     * flywheel speed. In the alliance zone the target is the closest hub corner;
     * outside the alliance zone it is the closest alliance zone corner.
     *
     * @param spindexerFeedCommand Command to feed the spindexer (composed
     *                             externally)
     * @param indexerFeedCommand   Command to feed the indexer (composed externally)
     * @return The shoot command (requires this Shooter subsystem)
     */
    public Command buildShootCommand(Command spindexerFeedCommand, Command indexerFeedCommand, ShooterMode shooterMode) {
        return run(() -> {
            var cmd = getCurrentSOTFCommand();

            double rpm = 0;
            double hoodAngleDeg = 0;
            double turretAngleDeg = 0;
            switch(shooterMode) {
                case AgainstHub:
                    hoodAngleDeg = 5.5;
                    turretAngleDeg = -90;
                    rpm = FlywheelSetpoint.AgainstHub.leaderMotorTarget.in(RotationsPerSecond);
                    break;
                case Pass:
                    hoodAngleDeg = 10;
                    turretAngleDeg = -90;
                    rpm = FlywheelSetpoint.Pass.leaderMotorTarget.in(RotationsPerSecond);
                    break;
                case Autoaim: {
                    hoodAngleDeg = cmd.hoodAngle() + hoodAngleOffsetDeg.get();
                    // Turret: continuously track the SOTF turret angle (robot-relative)
                    turretAngleDeg = cmd.turretAngle()
                            .minus(poseSupplier.get().getRotation())
                            .getDegrees();
                    rpm = cmd.rpm() + rpmOffset.get();

                    // Flywheel: set target speed
                    /*
                    if (isInAllianceZone()) {
                        flywheel.setTarget(rpm);
                    } else {
                        flywheel.setTarget(FlywheelSetpoint.Far.leaderMotorTarget.in(RotationsPerSecond));
                    } */
                }
            }

            hood.setAngle(hoodAngleDeg);
            turret.setAngle(turretAngleDeg);
            flywheel.setTarget(rpm);
        }).alongWith(
                // Spindexer + indexer: wait for flywheel to reach speed, then feed
                Commands.waitUntil(() -> (flywheel.isNearTarget(RotationsPerSecond.of(1))))
                        .andThen(
                                // Reset spawn timer so the first fuel fires immediately
                                Commands.runOnce(() -> fuelSpawnTimer.restart()),
                                Commands.parallel(
                                        spindexerFeedCommand,
                                        indexerFeedCommand,
                                        // Periodically spawn simulated fuel while feeding
                                        Commands.run(this::trySpawnSimulatedFuel))))
                .withName("Shoot");
    }
    
    /**
     * Build a tuning command: while held, set hood and flywheel to the
     * DogLog tunable values (treated as absolute), aim turret using SOTF
     * turret angle, and log the effective distance for LUT tuning.
     */
    public Command buildTuningCommand() {
        return run(() -> {
            var cmd = getCurrentSOTFCommand();

            // Treat DogLog tunables as absolute setpoints while tuning
            double rpm = rpmOffset.get();
            double hoodAngleDeg = hoodAngleOffsetDeg.get();

            // Turret: continuously track the SOTF turret angle (robot-relative)
            double turretAngleDeg = cmd.turretAngle()
                    .minus(poseSupplier.get().getRotation())
                    .getDegrees();
            turret.setAngle(turretAngleDeg);

            // Hood: set directly from tunable
            hood.setAngle(hoodAngleDeg);

            // Flywheel: set directly from tunable
            flywheel.setTarget(rpm);
        }).withName("SOTF-Tune");
    }

    /**
     * Command that sets the turret to a specific angle (one-shot).
     *
     * @param angleDegrees The target angle in degrees
     * @return Command requiring this subsystem
     */
    public Command setTurretAngleCommand(double angleDegrees) {
        return runOnce(() -> turret.setAngle(angleDegrees));
    }

    /**
     * Command that continuously follows a dynamic turret angle.
     *
     * @param angleDegreesSupplier Supplier returning the target angle in degrees
     * @return Command requiring this subsystem
     */
    public Command followTurretAngleCommand(Supplier<Double> angleDegreesSupplier) {
        return run(() -> turret.setAngle(angleDegreesSupplier.get()));
    }

    /**
     * Command that sets the hood to a specific angle (one-shot).
     *
     * @param angleDegrees The target angle in degrees
     * @return Command requiring this subsystem
     */
    public Command setHoodAngleCommand(double angleDegrees) {
        return runOnce(() -> hood.setAngle(angleDegrees));
    }

    /**
     * Command that continuously follows a dynamic hood angle.
     *
     * @param angleDegreesSupplier Supplier returning the target angle in degrees
     * @return Command requiring this subsystem
     */
    public Command followHoodAngleCommand(Supplier<Double> angleDegreesSupplier) {
        return run(() -> hood.setAngle(angleDegreesSupplier.get()));
    }

    /**
     * Command that sets the flywheel to a specific setpoint (one-shot).
     *
     * @param setpoint The flywheel setpoint enum
     * @return Command requiring this subsystem
     */
    public Command setFlywheelSetpointCommand(FlywheelSetpoint setpoint) {
        return runOnce(() -> flywheel.setTargetSetpoint(setpoint));
    }

    /**
     * Command that continuously drives the flywheel to a dynamic velocity.
     *
     * @param velocityRPSSupplier Supplier returning target velocity in rot/s
     * @return Command requiring this subsystem
     */
    public Command followFlywheelCommand(Supplier<Double> velocityRPSSupplier) {
        return run(() -> flywheel.setTarget(velocityRPSSupplier.get()));
    }

    /**
     * Command that coasts the flywheel (one-shot).
     *
     * @return Command requiring this subsystem
     */
    public Command coastFlywheelCommand() {
        return runOnce(() -> flywheel.coast());
    }

    // ── Telemetry / visualization accessors ──────────────────────────────

    /** @return 3D pose of the turret for mechanism visualization */
    public Pose3d getTurretMechanismPose() {
        return turret.getMechanismPose();
    }

    /** @return 3D pose of the hood for mechanism visualization */
    public Pose3d getHoodMechanismPose() {
        Rotation3d turretRotation = turret.getMechanismPose().getRotation();
        return hood.getMechanismPose(turretRotation);
    }

    /**
     * @return Whether the flywheel is near its target velocity
     * @param threshold The acceptable error threshold
     */
    public boolean isFlywheelNearTarget(AngularVelocity threshold) {
        return flywheel.isNearTarget(threshold);
    }

    public final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                    null, // Use default timeout (10 s)
                          // Log state with Phoenix SignalLogger class
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> flywheel.runSysid(volts),
                    null,
                    this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    public Command runReverse() {
        return Commands.runOnce(() -> {
            flywheel.setTarget(FlywheelSetpoint.Outtake.leaderMotorTarget.in(RotationsPerSecond));
        });
    }

    /**
   * Creates a command to stop the intake.
   * 
   * @return A command that stops the intake
   */
  public Command resetFlywheel() {
    return runOnce(() -> flywheel.setTarget(FlywheelSetpoint.AgainstHub.leaderMotorTarget.in(RotationsPerSecond)));
  }

}
