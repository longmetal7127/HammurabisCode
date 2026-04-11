package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.lang.reflect.Field;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;

import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    public final Turret turret = new Turret();
    public final Hood hood = new Hood();
    public final Flywheel flywheel = new Flywheel();

    public enum ShooterMode {
        /** Stop and Shoot Against Hub */
        AgainstHub,
        /** Pass across the field */
        Pass,
        /** Autoaim turret */
        Autoaim,
        Autonomous
    }

    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier;
    private final FuelSim fuelSim;

    private final ShootOnTheFlyCalculator sotfCalculator = buildSOTFCalculator();
    private final ShootOnTheFlyCalculator passingSotfCalculator = buildPassingSOTFCalculator();

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
    public boolean isInAllianceZone() {
        double robotX = poseSupplier.get().getX();
        var blue = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
        if (blue) {
            return robotX < FieldConstants.LinesVertical.allianceZone;
        } else {
            return robotX > FieldConstants.LinesVertical.oppAllianceZone;
        }
    }

    /**
     * Finds the nearest alliance-zone corner to the robot's current position.
     * Used when shooting from the neutral zone back toward our alliance zone.
     */
    private Translation2d getClosestAllianceZoneCorner() {
        Translation2d robot = poseSupplier.get().getTranslation();
        boolean blue = DriverStation.getAlliance().get() == Alliance.Blue;
        Translation2d[] corners = {
                blue ? FieldConstants.AllianceZoneCorners.blueLeft : FieldConstants.AllianceZoneCorners.redLeft,
                blue ? FieldConstants.AllianceZoneCorners.blueRight : FieldConstants.AllianceZoneCorners.redRight,
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
        Translation3d hub = DriverStation.getAlliance().get() == Alliance.Blue ? FieldConstants.Hub.topCenterPoint
                : FieldConstants.Hub.oppTopCenterPoint;
        Translation2d target = isInAllianceZone()
                ? new Translation2d(hub.getX(), hub.getY())
                : getClosestAllianceZoneCorner();
        ChassisSpeeds speeds = fieldRelativeSpeedsSupplier.get();

        // Pass the robot-relative offset to the calculator so it handles the omega*r
        // math itself
        ShootOnTheFlyCalculator activeCalculator = isInAllianceZone() ? sotfCalculator : passingSotfCalculator;

        ShootOnTheFlyCalculator.ShooterCommand cmd = activeCalculator.calculate(
                robotPose.getTranslation(),
                robotPose.getRotation(),
                new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                speeds.omegaRadiansPerSecond,
                target,
                0);
        DogLog.log("Shooter/SOTF/EffectiveDistanceMeters", cmd.effectiveDistance());
        return cmd;
    }

    /**
     * Builds the shoot-on-the-fly calculator with placeholder table entries.
     */
    private static ShootOnTheFlyCalculator buildSOTFCalculator() {
        ShootOnTheFlyCalculator calc = new ShootOnTheFlyCalculator();
        calc.addTableEntry(1.127, 1800, 4.4, 1.11666667);
        calc.addTableEntry(2.111, 32 * 60, 9.5, 1.23333333);
        calc.addTableEntry(3.002, 33 * 60, 13, 1.3);
        calc.addTableEntry(4.001, 36 * 60, 17, 1.33333334);
        calc.addTableEntry(4.977, 2298, 25, 1.292);
        calc.addTableEntry(5.980, 41 * 60, 30, 1.21829105);

        return calc;
    }

    /**
     * Builds the shoot-on-the-fly calculator with placeholder table entries.
     */
    private static ShootOnTheFlyCalculator buildPassingSOTFCalculator() {
        ShootOnTheFlyCalculator calc = new ShootOnTheFlyCalculator();
        calc.addTableEntry(3.048, 25 * 60, 30, 1.06844741);
        calc.addTableEntry(3.048 + 1.524, 30 * 60, 30, 1.2677231);
        calc.addTableEntry(3.048 + 1.524 + 1.524, 36 * 60, 30, 1.4676451);
        calc.addTableEntry(3.048 + 1.524 + 1.524 + 1.524, 41 * 60, 30, 1.63333333);
        calc.addTableEntry(3.048 + 1.524 + 1.524 + 1.524 + 1.524, 46 * 60, 30, 1.6844563);

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
        launchSimulatedFuelNow();
        fuelSpawnTimer.restart();
    }

    /** Launches one fuel projectile into FuelSim immediately (no rate-limit check). */
    private void launchSimulatedFuelNow() {
        double hoodPitchRad = Units.rotationsToRadians(hood.getPosition());

        // FuelSim expects robot-relative turret yaw where 0 deg points robot-forward.
        // Turret mechanism angle uses 0 deg = rear, so convert before launching.
        double turretYawRad = Math.toRadians(turretSetpointToRobotRelativeDeg(turret.getAngleDegrees()));

        double flywheelRPS = flywheel.getleaderMotorVelocity().in(RotationsPerSecond);
        double exitSpeedMps = Math.abs(flywheelRPS) * 2.0 * Math.PI * FLYWHEEL_RADIUS;
        System.out.println("Launching simulated fuel with exit speed " + exitSpeedMps + " m/s, hood pitch "
                + Math.toDegrees(hoodPitchRad) + " deg, turret yaw " + Math.toDegrees(turretYawRad) + " deg");
        fuelSim.launchFuel(
                MetersPerSecond.of(exitSpeedMps),
                Radians.of(hoodPitchRad),
                Radians.of(turretYawRad),
                Inches.of(22)); // launch height — above bumper height
    }

    // ── Commands ─────────────────────────────────────────────────────────
    public Trigger readyToShoot = new Trigger(() -> flywheel.isNearTarget(RotationsPerSecond.of(10))
            && Math.abs(turret.getAngleDegrees() - turret.setpoint) < 5.0);

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
    public Command buildShootCommand(Command spindexerFeedCommand, Command indexerFeedCommand,
            ShooterMode shooterMode, boolean turretOnly, Spindexer spindexer) {

    Command feedAndSimulateCommand = Commands.waitUntil(readyToShoot)
        .andThen(
            // Reset spawn timer so the first fuel fires immediately
            Commands.runOnce(() -> {
                if (RobotBase.isSimulation()) {
                launchSimulatedFuelNow();
                fuelSpawnTimer.restart();
                }
            }),
            Commands.parallel(
                spindexerFeedCommand,
                indexerFeedCommand,
                // Periodically spawn simulated fuel while feeding
                Commands.run(this::trySpawnSimulatedFuel)
            ).until(() -> !readyToShoot.getAsBoolean())
        ).repeatedly();

        return runEnd(() -> {
            var cmd = getCurrentSOTFCommand();

            double rpm = 0;
            double hoodAngleDeg = 0;
            double turretAngleDeg = 0;
            switch (shooterMode) {
                case AgainstHub:
                    hoodAngleDeg = 5.5;
                    turretAngleDeg = 90;
                    rpm = FlywheelSetpoint.AgainstHub.leaderMotorTarget.in(RotationsPerSecond);
                    break;
                case Pass:
                    hoodAngleDeg = 30;
                    turretAngleDeg = 90;
                    rpm = FlywheelSetpoint.Pass.leaderMotorTarget.in(RotationsPerSecond);
                    break;
                case Autonomous:
                    hoodAngleDeg = 5.5;
                    turretAngleDeg = 90;
                    rpm = FlywheelSetpoint.Autonomous.leaderMotorTarget.in(RotationsPerSecond);
                    break;
                case Autoaim: {
                    hoodAngleDeg = cmd.hoodAngle(); // hoodAngleOffsetDeg.get();
                    // Turret: continuously track the SOTF turret angle (robot-relative)
                    turretAngleDeg = cmd.turretAngle()
                            .minus(poseSupplier.get().getRotation())
                            .getDegrees();
                    rpm = cmd.rpm() / 60.0;// rpmOffset.get();
                }
            }
            if (!turretOnly) {
                hood.setAngle(hoodAngleDeg);
                flywheel.setTarget(rpm);
            }

            double turretFFRadPerSec = (shooterMode == ShooterMode.Autoaim) ? cmd.turretAngularVelocityFFRadPerSec()
                    : 0.0;
            turret.setAngle(robotRelativeToTurretSetpointDeg(turretAngleDeg), turretFFRadPerSec);
        }, () -> {
            hood.setAngle(0);
    }).alongWith(
        Commands.either(
            feedAndSimulateCommand,
            Commands.none(),
            () -> !turretOnly))
        .withName("Shoot");
        /* //automatically reversing spindexer if it jams
                        .andThen(Commands.either(
                                Commands.sequence(
                                        // Reset spawn timer so the first fuel fires immediately
                                        Commands.runOnce(() -> fuelSpawnTimer.restart()),
                                        Commands.parallel(
                                                spindexerFeedCommand,
                                                indexerFeedCommand,
                                                // Periodically spawn simulated fuel while feeding
                                                Commands.run(this::trySpawnSimulatedFuel))
                                                .until(() -> !readyToShoot.getAsBoolean()))
                                        .repeatedly(),
                                Commands.sequence(
                                        Commands.sequence(
                                                // Reset spawn timer so the first fuel fires immediately
                                                Commands.runOnce(() -> fuelSpawnTimer.restart()),
                                                Commands.parallel(
                                                        spindexerFeedCommand,
                                                        indexerFeedCommand,
                                                        // Periodically spawn simulated fuel while feeding
                                                        Commands.run(this::trySpawnSimulatedFuel))
                                                        .until(() -> !readyToShoot.getAsBoolean()))
                                                .until(spindexer.spindexerJamming),
                                        spindexer.setTargetTemporary(Spindexer.SpindexerSetpoint.SpinReverse)
                                                .withTimeout(1))
                                        .repeatedly(),
                                () -> (spindexer.equals(null)))))
                .withName("Shoot");*/
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
            double turretAngleDegRobotRel = cmd.turretAngle()
                    .minus(poseSupplier.get().getRotation())
                    .getDegrees();
            // Compensate because the turret's zero is pointing directly rearward.
            // Convert robot-relative angle (0 = forward) to turret mechanism angle (0 =
            // rear).
            turret.setAngle(robotRelativeToTurretSetpointDeg(turretAngleDegRobotRel));

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
        // Assume callers provide a robot-relative angle (0 = forward). Compensate for
        // turret zero.
        return runOnce(() -> turret.setAngle(robotRelativeToTurretSetpointDeg(angleDegrees)));
    }

    /**
     * Command that continuously follows a dynamic turret angle.
     *
     * @param angleDegreesSupplier Supplier returning the target angle in degrees
     * @return Command requiring this subsystem
     */
    public Command followTurretAngleCommand(Supplier<Double> angleDegreesSupplier) {
        // Supplier returns robot-relative angle (0 = forward). Compensate before
        // setting.
        return run(() -> turret.setAngle(robotRelativeToTurretSetpointDeg(angleDegreesSupplier.get())));
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
     * Convert a robot-relative turret angle (0 = robot forward, + right) to the
     * turret mechanism setpoint where 0 = turret pointing directly rearward.
     * This accounts for the turret's hardware zero being rear-facing.
     */
    private double robotRelativeToTurretSetpointDeg(double robotRelativeDeg) {
        double mechDeg = robotRelativeDeg + 180.0; // map forward(0) -> rear(180)
        return normalizeDegrees(mechDeg);
    }

    /**
     * Convert turret mechanism angle (0 = rear) back to robot-relative angle
     * (0 = forward), matching FuelSim's turret-yaw convention.
     */
    private double turretSetpointToRobotRelativeDeg(double turretMechDeg) {
        return normalizeDegrees(turretMechDeg - 180.0);
    }

    /** Normalize degrees to the range (-180, 180]. */
    private double normalizeDegrees(double deg) {
        while (deg > 180.0) {
            deg -= 360.0;
        }
        while (deg <= -180.0) {
            deg += 360.0;
        }
        return deg;
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
