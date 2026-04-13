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
      Rotation2d turretAngle, double rpm, double hoodAngle, double effectiveDistance, double turretAngularVelocityFFRadPerSec) {}

  private final InterpolatingTreeMap<Double, ShooterParams> shooterTable;
  
  /** Offset of the launcher from the center of the robot. */
  private Translation2d launcherOffset = new Translation2d(-0.1651, -0.1016);

  /** Previous-cycle solved TOF used as a warm-start seed. */
  private double previousTOF = -1.0;

  // Solver conditioning defaults (inspired by frc-fire-control ShotCalculator)
  private int maxIterations = 25;
  private double convergenceToleranceSec = 0.001;
  private double tofMinSec = 0.05;
  private double tofMaxSec = 3.0;
  private double minSotfSpeedMps = 0.1;
  private double maxSotfSpeedMps = 8.0;



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

  /** Clears warm-start state (recommended after pose resets or major state discontinuities). */
  public void resetWarmStart() {
    previousTOF = -1.0;
  }

  /** Optional solver tuning for iteration count and convergence tolerance. */
  public void setSolverTuning(int maxIterations, double convergenceToleranceSec) {
    this.maxIterations = Math.max(1, maxIterations);
    this.convergenceToleranceSec = Math.max(1e-6, convergenceToleranceSec);
  }

  /** Optional TOF bounds used for per-iteration clamping. */
  public void setTofBounds(double tofMinSec, double tofMaxSec) {
    this.tofMinSec = Math.max(1e-3, Math.min(tofMinSec, tofMaxSec));
    this.tofMaxSec = Math.max(this.tofMinSec, tofMaxSec);
  }

  /** Optional speed bounds for enabling/disabling SOTF compensation. */
  public void setSotfSpeedBounds(double minSotfSpeedMps, double maxSotfSpeedMps) {
    this.minSotfSpeedMps = Math.max(0.0, Math.min(minSotfSpeedMps, maxSotfSpeedMps));
    this.maxSotfSpeedMps = Math.max(this.minSotfSpeedMps, maxSotfSpeedMps);
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
    return calculate(robotPosition, new Rotation2d(), new Translation2d(), 0.0, goalPosition, 0.0);
  }

  /**
   * Legacy method for calculating shooter parameters without angular velocity data.
   */
  public ShooterCommand calculate(
      Translation2d robotPosition,
      Translation2d robotVelocity,
      Translation2d goalPosition,
      double latencyCompensation) {
    return calculate(
        robotPosition, new Rotation2d(), robotVelocity, 0.0, goalPosition, latencyCompensation);
  }

  /**
   * Calculates shooter parameters compensating for robot motion using Newton's method on the
   * time-of-flight residual. Now includes omega x r velocity correction and angular velocity feedforward.
   *
   * @param robotPosition Current robot position on field
   * @param robotHeading Current robot heading
   * @param robotVelocity Current translational robot velocity (field-relative, m/s)
   * @param robotOmegaRadPerSec Current angular velocity (rad/s)
   * @param goalPosition Goal/target position on field
   * @param latencyCompensation Additional time to project position forward (seconds)
   * @return Shooter command with turret angle, RPM, hood angle, and angular velocity feedforward
   */
  public ShooterCommand calculate(
      Translation2d robotPosition,
      Rotation2d robotHeading,
      Translation2d robotVelocity,
      double robotOmegaRadPerSec,
      Translation2d goalPosition,
      double latencyCompensation) {

    // 0. Compute omega x r launcher velocity correction
    Translation2d rotatedOffset = launcherOffset.rotateBy(robotHeading);
    
    // Effective field position of the launcher
    Translation2d launcherPosition = robotPosition.plus(rotatedOffset);
    
    // Effective field velocity of the launcher (v + omega x r)
    Translation2d effectiveVelocity = new Translation2d(
        robotVelocity.getX() - rotatedOffset.getY() * robotOmegaRadPerSec,
        robotVelocity.getY() + rotatedOffset.getX() * robotOmegaRadPerSec
    );

    // 1. Project future position (account for latency)
    Translation2d futurePos = launcherPosition.plus(effectiveVelocity.times(latencyCompensation));

    // 2. Static displacement to goal (before velocity compensation)
    Translation2d staticToGoal = goalPosition.minus(futurePos);
    double staticDistance = staticToGoal.getNorm();

    if (staticDistance < 1e-6) {
      previousTOF = -1.0;
      return new ShooterCommand(new Rotation2d(), 0.0, 0.0, 0.0, 0.0);
    }

    // 3. Compute the projectile speed v_p from the static lookup (used for proxy derivative)
    ShooterParams staticParams = shooterTable.get(staticDistance);
    if (staticParams == null || staticParams.timeOfFlight() <= 0.0) {
      previousTOF = -1.0;
      return new ShooterCommand(staticToGoal.getAngle(), 0.0, 0.0, staticDistance, 0.0);
    }

    double vp = staticDistance / staticParams.timeOfFlight();
    double robotSpeed = effectiveVelocity.getNorm();

    // Solution conditioning feature: for near-static speed, use plain LUT shot.
    if (robotSpeed < minSotfSpeedMps || robotSpeed > maxSotfSpeedMps) {
      System.out.println("exiting early");
      previousTOF = staticParams.timeOfFlight();
      return new ShooterCommand(
          staticToGoal.getAngle(),
          staticParams.rpm(),
          staticParams.hoodAngle(),
          staticDistance,
          0.0);
    }

    // 4. Initial TOF guess: τ_0 = D / (v_p + |v| * cos(θ))
    double cosTheta = 0.0;
    if (robotSpeed > 1e-6 && staticDistance > 1e-6) {
      cosTheta =
          (staticToGoal.getX() * effectiveVelocity.getX()
                  + staticToGoal.getY() * effectiveVelocity.getY())
              / (staticDistance * robotSpeed);
    }

    // Warm start when available; fallback to projected geometric guess.
    double tau;
    if (previousTOF > 0.0) {
      tau = previousTOF;
    } else {
      double initialDenominator = vp + robotSpeed * cosTheta;
      if (initialDenominator <= 1e-3) {
        tau = staticParams.timeOfFlight();
      } else {
        tau = staticDistance / initialDenominator;
      }
    }
    tau = MathUtil.clamp(tau, tofMinSec, tofMaxSec);

    // 5. Newton iteration on the TOF residual: E(τ) = τ - τ_LUT(D(τ))
    for (int i = 0; i < maxIterations; i++) {
      Translation2d d = staticToGoal.minus(effectiveVelocity.times(tau));
      double D = d.getNorm();

      if (D < 0.01) {
        tau = staticParams.timeOfFlight();
        break;
      }

      ShooterParams params = shooterTable.get(D);
      double tauLUT = params != null && params.timeOfFlight() > 0.0 ? params.timeOfFlight() : tau;
      double E = tau - tauLUT;

      if (Math.abs(E) < convergenceToleranceSec) {
        break;
      }

      double dDotV = d.getX() * effectiveVelocity.getX() + d.getY() * effectiveVelocity.getY();
      if (Math.abs(vp * D) < 1e-6) {
        tau = tauLUT;
        break;
      }
      double EPrime = 1.0 + dDotV / (vp * D);

      if (Math.abs(EPrime) > 0.01) {
        tau -= E / EPrime;
      } else {
        // Near-singular derivative: fallback to fixed-point step.
        tau = tauLUT;
      }

      // Per-iteration clamp to avoid runaway / branch jumps.
      tau = MathUtil.clamp(tau, tofMinSec, tofMaxSec);
    }

    if (!Double.isFinite(tau)) {
      tau = staticParams.timeOfFlight();
    }
    previousTOF = tau;

    // 6. Final solution: compute virtual target at converged TOF
    Translation2d d = staticToGoal.minus(effectiveVelocity.times(tau));
    double effectiveDistance = d.getNorm();
    Rotation2d turretAngle = effectiveDistance > 1e-6 ? d.getAngle() : staticToGoal.getAngle();
    
  // 7. Angular velocity feedforward: rate of change of aim angle
  // Use converged geometry (d/effectiveDistance) so FF matches final solution.
    double angularVelocityFF = 0.0;
  if (effectiveDistance > 0.1) {
    double tangentialVel = (d.getY() * effectiveVelocity.getX() - d.getX() * effectiveVelocity.getY()) / effectiveDistance;
    angularVelocityFF = (tangentialVel / effectiveDistance) - robotOmegaRadPerSec;
    }

    // 8. Look up RPM and hood angle at the effective distance
  ShooterParams finalParams = shooterTable.get(effectiveDistance);
  double outRpm = finalParams != null ? finalParams.rpm() : staticParams.rpm();
  double outHood = finalParams != null ? finalParams.hoodAngle() : staticParams.hoodAngle();

    return new ShooterCommand(
        turretAngle, outRpm, outHood, effectiveDistance, angularVelocityFF);
  }
}
