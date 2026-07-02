package frc.robot.subsystems.ShooterV2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.ShooterV2.*;;

/**
 * ShootingCalculator
 * ─────────────────────────────────────────────────────────────────────────────
 * Pure-physics shooting calculator.  Given the robot's field-relative pose
 * and velocity, it computes the exact turret angle, hood angle, and flywheel
 * RPS needed to score a ball in the hub — including full moving-shot prediction.
 *
 * ┌─────────────────────────────────────────────────────────────┐
 * │  COORDINATE SYSTEM                                          │
 * │  • Field X  →  pointing toward the red alliance wall        │
 * │  • Field Y  →  pointing toward the top of the field        │
 * │  • Robot heading 0°  →  facing positive X (red wall)       │
 * │  • Turret angle 0°   →  aligned with robot forward         │
 * │  • Hood angle 0°     →  flat / horizontal                  │
 * └─────────────────────────────────────────────────────────────┘
 *
 * How the physics work
 * ─────────────────────
 * 1.  We compute a "virtual target" by dead-reckoning the robot forward by
 *     SHOT_LATENCY_S to account for processing + spin-up delay.
 * 2.  From the shooter's 3-D exit point to the hub centre we solve the
 *     projectile equations iteratively, including aerodynamic drag.
 * 3.  A correction is applied so that if the robot is moving, the ball's
 *     horizontal velocity (in the field frame) is the vector sum of the
 *     flywheel-imparted velocity and the robot's velocity.
 * ─────────────────────────────────────────────────────────────────────────────
 */
public class ShootingCalculator {

    // ── Result container ─────────────────────────────────────────────────────

    /**
     * Immutable snapshot of a single shot solution.
     * All angles in degrees, velocity in RPS.
     */
    public static final class ShotSolution {

        /** Turret angle relative to robot forward, degrees. CCW positive. */
        public final double turretAngleDeg;

        /** Hood elevation angle, degrees. */
        public final double hoodAngleDeg;

        /** Flywheel speed, rotations per second. */
        public final double flywheelRPS;

        /** Predicted ball flight time, seconds (informational). */
        public final double flightTimeS;

        /** Horizontal distance from shooter to hub at moment of shot, meters. */
        public final double distanceM;

        /** Whether this solution is physically achievable with current hardware limits. */
        public final boolean isFeasible;

        public ShotSolution(
                double turretAngleDeg,
                double hoodAngleDeg,
                double flywheelRPS,
                double flightTimeS,
                double distanceM,
                boolean isFeasible) {
            this.turretAngleDeg = turretAngleDeg;
            this.hoodAngleDeg   = hoodAngleDeg;
            this.flywheelRPS    = flywheelRPS;
            this.flightTimeS    = flightTimeS;
            this.distanceM      = distanceM;
            this.isFeasible     = isFeasible;
        }

        @Override
        public String toString() {
            return String.format(
                "ShotSolution{turret=%.1f°, hood=%.1f°, rps=%.1f, t=%.3fs, d=%.2fm, ok=%b}",
                turretAngleDeg, hoodAngleDeg, flywheelRPS, flightTimeS, distanceM, isFeasible);
        }
    }

    // ── Internal drag constant (precomputed for speed) ────────────────────────

    /**
     * k = 0.5 * Cd * A * rho
     * Used in:  F_drag = k * v²   (magnitude)
     */
    private static final double DRAG_K =
            0.5
            * ShooterConstants.BALL_DRAG_COEFF
            * ShooterConstants.BALL_CROSS_SECTION_M2
            * ShooterConstants.AIR_DENSITY_KG_M3;

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Calculate the full shot solution for the current robot state.
     *
     * @param robotPose     Current field-relative robot pose (from fused odometry/vision)
     * @param robotSpeeds   Current chassis speeds in the field frame (field-relative ChassisSpeeds)
     * @return              ShotSolution with all outputs, or an infeasible solution if the
     *                      shot cannot be made from this position/speed.
     */
    public static ShotSolution calculate(Pose2d robotPose, ChassisSpeeds robotSpeeds) {

        // 1. Pick the correct hub for our alliance
        Translation2d hubPos = getHubPosition();

        // 2. Compute the shooter's actual field position (offset from robot center)
        Translation2d shooterPos = getShooterFieldPosition(robotPose);

        // 3. Predict where the robot will be when the ball actually exits
        //    (compensate for latency: vision pipeline + command loop + spin-up)
        Translation2d predictedShooterPos = predictShooterPosition(
                shooterPos, robotSpeeds, ShooterConstants.SHOT_LATENCY_S);

        // 4. Vector from predicted shooter position to hub (horizontal plane)
        Translation2d toHub = hubPos.minus(predictedShooterPos);
        double horizontalDist = toHub.getNorm();

        // 5. Height delta: hub opening minus shooter exit height
        double deltaHeight = ShooterConstants.HUB_HEIGHT_M - ShooterConstants.SHOOTER_HEIGHT_M;

        // 6. Solve ballistics: find hood angle + launch speed so ball reaches hub
        double[] ballisticResult = solveBallisticIterative(
                horizontalDist, deltaHeight, robotSpeeds, toHub);

        if (ballisticResult == null) {
            // No valid solution found (too far, too close, or speed limit exceeded)
            return new ShotSolution(0, ShooterConstants.HOOD_MIN_DEG,
                    ShooterConstants.FLYWHEEL_MIN_RPS, 0, horizontalDist, false);
        }

        double launchAngleDeg = ballisticResult[0];
        double launchSpeedMS  = ballisticResult[1];
        double flightTimeS    = ballisticResult[2];

        // 7. Convert launch speed (m/s, ball surface) → flywheel RPS
        //    ballSpeed = flywheelSurfaceSpeed * efficiency
        //    flywheelSurfaceSpeed = 2π * radius * RPS
        double flywheelRPS = launchSpeedMS
                / (ShooterConstants.FLYWHEEL_EFFICIENCY
                   * 2.0 * Math.PI * ShooterConstants.FLYWHEEL_RADIUS_M);

        // 8. Compute turret angle
        //    Direction to hub in field frame:
        double fieldAngleToHubDeg = Math.toDegrees(Math.atan2(toHub.getY(), toHub.getX()));
        //    Robot heading in field frame:
        double robotHeadingDeg = robotPose.getRotation().getDegrees();
        //    Turret angle = difference, normalised to (-180, 180]
        double turretAngleDeg = normalizeAngle(fieldAngleToHubDeg - robotHeadingDeg);

        // 9. Clamp hood angle to mechanical limits
        double hoodAngleDeg = Math.max(ShooterConstants.HOOD_MIN_DEG,
                               Math.min(ShooterConstants.HOOD_MAX_DEG, launchAngleDeg));

        // 10. Feasibility checks
        boolean feasible = checkFeasibility(turretAngleDeg, hoodAngleDeg,
                flywheelRPS, launchAngleDeg);

        return new ShotSolution(
                turretAngleDeg,
                hoodAngleDeg,
                flywheelRPS,
                flightTimeS,
                horizontalDist,
                feasible);
    }

    // ── Ballistic Solver ──────────────────────────────────────────────────────

    /**
     * Iterative ballistic solver with aerodynamic drag and moving-shot correction.
     *
     * Strategy:
     *   For each candidate hood angle (coarse sweep, then refine):
     *     • Compute the required launch speed via RK4 numerical integration
     *       so that the ball lands exactly at (horizontalDist, deltaHeight).
     *     • Apply moving-shot correction: subtract the robot's velocity component
     *       along the shot direction from the required ball speed, so the ball's
     *       field-frame trajectory still hits the hub.
     *   Return the angle+speed pair that minimises overshoot while staying
     *   within flywheel limits.
     *
     * @param horizontalDist  Horizontal distance to hub, meters
     * @param deltaHeight     Hub height minus shooter height, meters
     * @param robotSpeeds     Field-relative chassis speeds for moving-shot correction
     * @param toHubVec        Unit direction vector toward hub (for velocity projection)
     * @return  [launchAngleDeg, launchSpeedMS, flightTimeS] or null if no solution
     */
    private static double[] solveBallisticIterative(
            double horizontalDist,
            double deltaHeight,
            ChassisSpeeds robotSpeeds,
            Translation2d toHubVec) {

        // Robot velocity projected onto the shot direction (m/s)
        double toHubUnitX = toHubVec.getX() / toHubVec.getNorm();
        double toHubUnitY = toHubVec.getY() / toHubVec.getNorm();
        double robotVelAlongShot = robotSpeeds.vxMetersPerSecond * toHubUnitX
                                 + robotSpeeds.vyMetersPerSecond * toHubUnitY;

        double bestAngle  = -1;
        double bestSpeed  = -1;
        double bestTime   = -1;
        double bestError  = Double.MAX_VALUE;

        // Sweep hood angles in 0.5° steps, find which angle minimises positional error
        for (double angleDeg = ShooterConstants.HOOD_MIN_DEG;
             angleDeg <= ShooterConstants.HOOD_MAX_DEG;
             angleDeg += 0.5) {

            double angleRad = Math.toRadians(angleDeg);

            // Binary-search for the launch speed that gets the ball to
            // horizontalDist at height deltaHeight for this angle.
            double lo = 1.0, hi = 60.0;   // m/s search range
            double foundSpeed = -1;
            double foundTime  = -1;

            for (int iter = 0; iter < 60; iter++) {
                double mid = (lo + hi) / 2.0;

                // Decompose launch velocity
                // Horizontal component is REDUCED by robot's velocity along shot
                // so the ball's field-frame horizontal = mid*cos(angle)
                // (the robot's motion adds free horizontal momentum)
                double vx0 = mid * Math.cos(angleRad) - robotVelAlongShot;
                double vy0 = mid * Math.sin(angleRad);

                double[] landing = simulateFlight(vx0, vy0, horizontalDist);
                double landHeight = landing[0];
                double landTime   = landing[1];

                double error = landHeight - deltaHeight;

                if (Math.abs(error) < 0.005) {   // 5 mm convergence
                    foundSpeed = mid;
                    foundTime  = landTime;
                    break;
                }

                // If ball lands too low, we need more speed
                if (error < 0) hi = mid; else lo = mid;
            }

            if (foundSpeed < 0) continue;   // no convergence for this angle

            // Score: prefer solutions closer to the centre of the flywheel's range
            double rps = foundSpeed
                    / (ShooterConstants.FLYWHEEL_EFFICIENCY
                       * 2.0 * Math.PI * ShooterConstants.FLYWHEEL_RADIUS_M);

            if (rps < ShooterConstants.FLYWHEEL_MIN_RPS
                    || rps > ShooterConstants.FLYWHEEL_MAX_RPS) continue;

            // Prefer the angle that keeps the shot steep enough to fall INTO the hub
            // (higher angles = steeper descent = larger effective target)
            // We score by minimising deviation from an "ideal" 45–60° window
            double idealAngle = 52.0;
            double score = Math.abs(angleDeg - idealAngle);

            if (score < bestError) {
                bestError = score;
                bestAngle = angleDeg;
                bestSpeed = foundSpeed;
                bestTime  = foundTime;
            }
        }

        if (bestAngle < 0) return null;
        return new double[]{ bestAngle, bestSpeed, bestTime };
    }

    /**
     * RK4 numerical integration of ball flight with aerodynamic drag.
     *
     * State vector:  [ x (horizontal), y (vertical), vx, vy ]
     *
     * @param vx0            Initial horizontal velocity (field-frame minus robot vel), m/s
     * @param vy0            Initial vertical velocity, m/s
     * @param targetX        Horizontal distance to stop at (hub distance), meters
     * @return               [ heightAtTargetX, timeOfFlight ]
     */
    private static double[] simulateFlight(double vx0, double vy0, double targetX) {
        double dt = ShooterConstants.SIM_TIMESTEP_S;
        double x = 0, y = 0, vx = vx0, vy = vy0;
        double t = 0;

        while (t < ShooterConstants.MAX_FLIGHT_TIME_S && x < targetX) {
            // Drag acceleration magnitude: a_drag = (k / m) * v²
            double speed   = Math.sqrt(vx * vx + vy * vy);
            double aDragMag = (DRAG_K / ShooterConstants.BALL_MASS_KG) * speed * speed;

            // Drag direction opposes velocity
            double ax = -aDragMag * (vx / speed);
            double ay = -ShooterConstants.GRAVITY_M_S2 - aDragMag * (vy / speed);

            // Simple RK4 for this timestep
            double k1vx = ax,        k1vy = ay;
            double k1x  = vx,        k1y  = vy;

            double vx2 = vx + 0.5*dt*k1vx, vy2 = vy + 0.5*dt*k1vy;
            double speed2 = Math.sqrt(vx2*vx2 + vy2*vy2) + 1e-9;
            double aD2  = (DRAG_K / ShooterConstants.BALL_MASS_KG) * speed2 * speed2;
            double k2vx = -aD2*(vx2/speed2),        k2vy = -ShooterConstants.GRAVITY_M_S2 - aD2*(vy2/speed2);
            double k2x  = vx2,                       k2y  = vy2;

            double vx3 = vx + 0.5*dt*k2vx, vy3 = vy + 0.5*dt*k2vy;
            double speed3 = Math.sqrt(vx3*vx3 + vy3*vy3) + 1e-9;
            double aD3  = (DRAG_K / ShooterConstants.BALL_MASS_KG) * speed3 * speed3;
            double k3vx = -aD3*(vx3/speed3),        k3vy = -ShooterConstants.GRAVITY_M_S2 - aD3*(vy3/speed3);
            double k3x  = vx3,                       k3y  = vy3;

            double vx4 = vx + dt*k3vx, vy4 = vy + dt*k3vy;
            double speed4 = Math.sqrt(vx4*vx4 + vy4*vy4) + 1e-9;
            double aD4  = (DRAG_K / ShooterConstants.BALL_MASS_KG) * speed4 * speed4;
            double k4vx = -aD4*(vx4/speed4),        k4vy = -ShooterConstants.GRAVITY_M_S2 - aD4*(vy4/speed4);
            double k4x  = vx4,                       k4y  = vy4;

            vx += dt/6.0*(k1vx + 2*k2vx + 2*k3vx + k4vx);
            vy += dt/6.0*(k1vy + 2*k2vy + 2*k3vy + k4vy);
            x  += dt/6.0*(k1x  + 2*k2x  + 2*k3x  + k4x );
            y  += dt/6.0*(k1y  + 2*k2y  + 2*k3y  + k4y );
            t  += dt;
        }

        return new double[]{ y, t };
    }

    // ── Helper Methods ────────────────────────────────────────────────────────

    /**
     * Returns the hub Translation2d for the current alliance.
     * Defaults to blue if alliance is unknown.
     */
    public static Translation2d getHubPosition() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return ShooterConstants.RED_HUB_POSITION;
        }
        return ShooterConstants.BLUE_HUB_POSITION;
    }

    /**
     * Computes the shooter's field-relative position, accounting for the
     * robot's rotational offset of the shooter mount.
     */
    private static Translation2d getShooterFieldPosition(Pose2d robotPose) {
        double headingRad = robotPose.getRotation().getRadians();
        double dx = ShooterConstants.SHOOTER_FORWARD_OFFSET_M * Math.cos(headingRad)
                  - ShooterConstants.SHOOTER_LATERAL_OFFSET_M * Math.sin(headingRad);
        double dy = ShooterConstants.SHOOTER_FORWARD_OFFSET_M * Math.sin(headingRad)
                  + ShooterConstants.SHOOTER_LATERAL_OFFSET_M * Math.cos(headingRad);
        return robotPose.getTranslation().plus(new Translation2d(dx, dy));
    }

    /**
     * Dead-reckons the shooter position forward by `dt` seconds using
     * the robot's current field-relative velocity.
     */
    private static Translation2d predictShooterPosition(
            Translation2d currentPos, ChassisSpeeds fieldSpeeds, double dt) {
        return new Translation2d(
                currentPos.getX() + fieldSpeeds.vxMetersPerSecond * dt,
                currentPos.getY() + fieldSpeeds.vyMetersPerSecond * dt);
    }

    /**
     * Normalises an angle to the range (-180, 180].
     */
    private static double normalizeAngle(double deg) {
        deg = deg % 360.0;
        if (deg > 180.0)  deg -= 360.0;
        if (deg <= -180.0) deg += 360.0;
        return deg;
    }

    /**
     * Checks whether the computed solution is within all hardware limits.
     */
    private static boolean checkFeasibility(
            double turretDeg, double hoodDeg, double rps, double rawAngleDeg) {
        if (turretDeg < ShooterConstants.TURRET_MAX_CCW_DEG
                || turretDeg > ShooterConstants.TURRET_MAX_CW_DEG) return false;
        if (rawAngleDeg < ShooterConstants.HOOD_MIN_DEG
                || rawAngleDeg > ShooterConstants.HOOD_MAX_DEG) return false;
        if (rps < ShooterConstants.FLYWHEEL_MIN_RPS
                || rps > ShooterConstants.FLYWHEEL_MAX_RPS) return false;
        return true;
    }

    /**
     * Convenience: returns true when all three outputs are within their
     * respective tolerances — i.e. the robot is ready to fire.
     *
     * @param currentTurretDeg  Current turret encoder position, degrees
     * @param currentHoodDeg    Current hood encoder position, degrees
     * @param currentRPS        Current flywheel velocity, RPS
     * @param solution          Desired shot solution
     */
    public static boolean isReadyToShoot(
            double currentTurretDeg,
            double currentHoodDeg,
            double currentRPS,
            ShotSolution solution) {
        if (!solution.isFeasible) return false;
        return Math.abs(currentTurretDeg - solution.turretAngleDeg)
                        <= ShooterConstants.TURRET_TOLERANCE_DEG
            && Math.abs(currentHoodDeg  - solution.hoodAngleDeg)
                        <= ShooterConstants.HOOD_TOLERANCE_DEG
            && Math.abs(currentRPS      - solution.flywheelRPS)
                        <= ShooterConstants.FLYWHEEL_TOLERANCE_RPS;
    }
}