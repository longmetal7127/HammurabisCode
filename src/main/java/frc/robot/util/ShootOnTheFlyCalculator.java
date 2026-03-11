package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * Calculator for shooting on the fly (SOTF). Compensates for robot velocity by adjusting turret
 * angle, flywheel RPM, and hood angle.
 *
 * <p>Based on the time-of-flight lookup table approach described at
 * blog.eeshwark.com/robotblog/shooting-on-the-fly-pt2
 * 
 * Initially implemented by team 4322, modified by 7127.
 */
public class ShootOnTheFlyCalculator {

  /** Parameters stored in the shooter lookup table. */
  public record ShooterParams(double rpm, double hoodAngle, double timeOfFlight) {}

  /** Output command from the calculator. */
  public record ShooterCommand(
      Rotation2d turretAngle, double rpm, double hoodAngle, double effectiveDistance) {}

  private final InterpolatingTreeMap<Double, ShooterParams> shooterTable;

  /**
   * Creates a calculator with the provided shooter table.
   *
   * @param shooterTable Interpolating map from distance (meters) to shooter parameters
   */
  public ShootOnTheFlyCalculator(InterpolatingTreeMap<Double, ShooterParams> shooterTable) {
    this.shooterTable = shooterTable;
  }

  /** Creates a calculator with a new empty table. Use {@link #addTableEntry} to populate. */
  public ShootOnTheFlyCalculator() {
    this.shooterTable =
        new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            (start, end, t) ->
                new ShooterParams(
                    MathUtil.interpolate(start.rpm(), end.rpm(), t),
                    MathUtil.interpolate(start.hoodAngle(), end.hoodAngle(), t),
                    MathUtil.interpolate(start.timeOfFlight(), end.timeOfFlight(), t)));
  }

  /**
   * Adds an entry to the shooter table.
   *
   * @param distance Distance in meters
   * @param rpm Flywheel RPM at this distance
   * @param hoodAngle Hood angle in degrees at this distance
   * @param timeOfFlight Time of flight in seconds at this distance
   */
  public void addTableEntry(double distance, double rpm, double hoodAngle, double timeOfFlight) {
    shooterTable.put(distance, new ShooterParams(rpm, hoodAngle, timeOfFlight));
  }

  /**
   * Calculates shooter parameters for a stationary robot.
   *
   * @param robotPosition Current robot position on field
   * @param goalPosition Goal/target position on field
   * @return Shooter command with turret angle, RPM, and hood angle
   */
  public ShooterCommand calculateStationary(
      Translation2d robotPosition, Translation2d goalPosition) {
    return calculate(robotPosition, new Translation2d(), goalPosition, 0.0);
  }

  /**
   * Calculates shooter parameters compensating for robot motion using Newton's method on the
   * time-of-flight residual.
   *
   *
   * @param robotPosition Current robot position on field
   * @param robotVelocity Current robot velocity (field-relative, m/s)
   * @param goalPosition Goal/target position on field
   * @param latencyCompensation Additional time to project position forward (seconds)
   * @return Shooter command with turret angle, RPM, and hood angle
   * @see <a
   *     href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/fire-control/newton-shooting.html">Newton's
   *     Method for Dynamic Shooting</a>
   */
  public ShooterCommand calculate(
      Translation2d robotPosition,
      Translation2d robotVelocity,
      Translation2d goalPosition,
      double latencyCompensation) {

    // 1. Project future position (account for latency)
    Translation2d futurePos = robotPosition.plus(robotVelocity.times(latencyCompensation));

    // 2. Static displacement to goal (before velocity compensation)
    Translation2d staticToGoal = goalPosition.minus(futurePos);
    double staticDistance = staticToGoal.getNorm();

    // 3. Compute the projectile speed v_p from the static lookup (used for proxy derivative)
    ShooterParams staticParams = shooterTable.get(staticDistance);
    double vp = staticDistance / staticParams.timeOfFlight();

    // 4. Initial TOF guess: τ_0 = D / (v_p + |v| * cos(θ))
    //    This accounts for the robot velocity component toward the target and avoids
    //    converging to the wrong root at high speeds.
    double robotSpeed = robotVelocity.getNorm();
    double cosTheta = 0.0;
    if (robotSpeed > 1e-6 && staticDistance > 1e-6) {
      // cos(θ) = dot(toGoal, velocity) / (|toGoal| * |velocity|)
      cosTheta =
          (staticToGoal.getX() * robotVelocity.getX()
                  + staticToGoal.getY() * robotVelocity.getY())
              / (staticDistance * robotSpeed);
    }
    double tau = staticDistance / (vp + robotSpeed * cosTheta);

    // 5. Newton iteration on the TOF residual: E(τ) = τ - τ_LUT(D(τ))
    for (int i = 0; i < 10; i++) {
      // Step 1: Virtual target displacement at current TOF guess
      Translation2d d = staticToGoal.minus(robotVelocity.times(tau));
      double D = d.getNorm();

      // Step 2: Residual — look up the table TOF at this distance
      ShooterParams params = shooterTable.get(D);
      double tauLUT = params.timeOfFlight();
      double E = tau - tauLUT;

      // Check convergence (within ~5ms)
      if (Math.abs(E) < 0.005) {
        break;
      }

      // Step 3 & 4: Proxy derivative E' = 1 + (d·v) / (v_p * D)
      //   where dD/dτ = -(d·v)/D, and τ'(D) ≈ 1/v_p (constant-velocity proxy)
      double dDotV = d.getX() * robotVelocity.getX() + d.getY() * robotVelocity.getY();
      double EPrime = 1.0 + dDotV / (vp * D);

      // Step 5: Newton update
      tau -= E / EPrime;
    }

    // 6. Final solution: compute virtual target at converged TOF
    Translation2d d = staticToGoal.minus(robotVelocity.times(tau));
    double effectiveDistance = d.getNorm();
    Rotation2d turretAngle = d.getAngle();

    // 7. Look up RPM and hood angle at the effective distance
    ShooterParams finalParams = shooterTable.get(effectiveDistance);

    return new ShooterCommand(
        turretAngle, finalParams.rpm(), finalParams.hoodAngle(), effectiveDistance);
  }
}
