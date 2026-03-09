package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Flywheel.FlywheelSetpoint;
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
    private final Turret turret = new Turret();
    @Logged
    private final Hood hood = new Hood();
    @Logged
    private final Flywheel flywheel = new Flywheel();

    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier;

    private final ShootOnTheFlyCalculator sotfCalculator = buildSOTFCalculator();


    /**
     * @param poseSupplier               Supplies the robot's current field pose
     * @param fieldRelativeSpeedsSupplier Supplies the robot's field-relative chassis speeds
     */
    public Shooter(
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.fieldRelativeSpeedsSupplier = fieldRelativeSpeedsSupplier;
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
        Translation2d robot = poseSupplier.get().getTranslation();
        Translation2d target = isInAllianceZone()
                ? getClosestHubCorner()
                : getClosestAllianceZoneCorner();
        ChassisSpeeds speeds = fieldRelativeSpeedsSupplier.get();
        return sotfCalculator.calculate(
                robot,
                new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                target,
                0.0);
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

    // ── Commands ─────────────────────────────────────────────────────────

    /**
     * Builds a command that continuously aims and shoots while held.
     *
     * <p>The SOTF calculator is always used for turret angle, hood angle, and
     * flywheel speed. In the alliance zone the target is the closest hub corner;
     * outside the alliance zone it is the closest alliance zone corner.
     *
     * @param spindexerFeedCommand Command to feed the spindexer (composed externally)
     * @param indexerFeedCommand   Command to feed the indexer (composed externally)
     * @return The shoot command (requires this Shooter subsystem)
     */
    public Command buildShootCommand(Command spindexerFeedCommand, Command indexerFeedCommand) {
        return run(() -> {
            var cmd = getCurrentSOTFCommand();

            // Turret: continuously track the SOTF turret angle (robot-relative)
            double turretAngleDeg = cmd.turretAngle()
                    .minus(poseSupplier.get().getRotation())
                    .getDegrees();
            turret.setAngle(turretAngleDeg);

            // Hood: follow the SOTF hood angle
            hood.setAngle(cmd.hoodAngle());

            // Flywheel: set target speed
            if (isInAllianceZone()) {
                flywheel.setTarget(cmd.rpm());
            } else {
                flywheel.setTarget(FlywheelSetpoint.Far.leaderMotorTarget.in(RotationsPerSecond));
            }
        }).alongWith(
                // Spindexer + indexer: wait for flywheel to reach speed, then feed
                Commands.waitUntil(() -> flywheel.isNearTarget(RotationsPerSecond.of(5)))
                        .andThen(Commands.parallel(
                                spindexerFeedCommand,
                                indexerFeedCommand))
        ).withName("Shoot");
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
}
