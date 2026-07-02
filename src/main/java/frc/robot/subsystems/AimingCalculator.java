package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldTargets;
import frc.robot.subsystems.drive.Drive;

/**
 * Pure physics aiming calculator.
 *
 * Solves the full shooting problem iteratively:
 *   distance -> hood angle -> exit velocity -> TOF -> virtual target -> repeat
 *
 * No lookup tables. All math is projectile physics.
 * Call solve() once per loop from your command, then pass the AimSolution
 * to each subsystem's applySolution() method.
 *
 * Coordinate conventions:
 *   - Hood angle: 0 deg = horizontal, positive = tilting up toward target
 *   - Shooter RPS: negative (your motor is inverted Clockwise_Positive, forward shot = negative RPS)
 *   - Turret motor rotations: 30:1 gear ratio, +/- 180 deg of travel
 *   - Hub height: measured from floor to hub center (TARGET_HEIGHT_M)
 *   - Shooter pivot height: measured from floor (SHOOTER_HEIGHT_M)
 */
public class AimingCalculator {

    // -----------------------------------------------------------------------
    // Physical constants — MEASURE THESE ON YOUR ROBOT
    // -----------------------------------------------------------------------

    /** Height of the hub target center above the floor (meters). */
    private static final double TARGET_HEIGHT_M = 2.64;

    /** Height of the shooter pivot above the floor (meters). */
    private static final double SHOOTER_HEIGHT_M = 0.315;

    /** Vertical drop the ball must travel (positive = up). */
    private static final double DELTA_HEIGHT_M = TARGET_HEIGHT_M - SHOOTER_HEIGHT_M;

    /** Flywheel radius (meters). 4 inch wheel = 0.0508m radius. Matches Shooter.getExitVelocity(). */
    private static final double FLYWHEEL_RADIUS_M = 0.0508;

    /** Gravity (m/s^2). */
    private static final double G = 9.80665;

    // -----------------------------------------------------------------------
    // Hood constraints
    // -----------------------------------------------------------------------

    /** Hood travel in ticks (matches Hood.MAX_TICKS / MIN_TICKS). */
    private static final double HOOD_MIN_TICKS = 0.0;
    private static final double HOOD_MAX_TICKS = 3.0;

    /** Ticks per degree (from Hood — this is your real encoder scale, kept as-is). */
    private static final double HOOD_TICKS_PER_DEGREE = 0.1339863933343103;

    /**
     * Real physical angle at HOOD_MIN_TICKS (0 ticks).
     * Measured on the robot: hood at minimum tick position sits at 75.2 degrees
     * from horizontal (this hood is a steep/near-vertical shooter, not a flat one).
     */
    private static final double HOOD_ANGLE_AT_MIN_TICKS_DEG = 75.2;

    /**
     * Hood physical angle range — offset by the real measured minimum angle.
     * HOOD_TICKS_PER_DEGREE still defines the scale (ticks per degree of motion),
     * but the absolute range is anchored to where the hood actually sits, not to 0.
     */
    private static final double HOOD_MIN_DEG = HOOD_ANGLE_AT_MIN_TICKS_DEG;
    private static final double HOOD_MAX_DEG =
            HOOD_ANGLE_AT_MIN_TICKS_DEG + (HOOD_MAX_TICKS - HOOD_MIN_TICKS) / HOOD_TICKS_PER_DEGREE;
    // NOTE: with HOOD_TICKS_PER_DEGREE = 0.134, this currently computes to ~97.6 deg,
    // wider than your measured 87.5 deg max. If 87.5 deg is the real hard stop,
    // either HOOD_MAX_TICKS or HOOD_TICKS_PER_DEGREE doesn't match the as-built
    // mechanism — worth re-measuring ticks-at-87.5-deg directly on the robot.
    // For now we additionally hard-clamp output to 87.5 so we never command past it:
    private static final double HOOD_HARD_MAX_DEG = 87.5;

    // -----------------------------------------------------------------------
    // Shooter constraints
    // -----------------------------------------------------------------------

    private static final double SHOOTER_MIN_RPS = 0.0;
    private static final double SHOOTER_MAX_RPS = 100.0;

    // -----------------------------------------------------------------------
    // Turret constants
    // -----------------------------------------------------------------------

    /** Motor rotations per full turret revolution (30:1 gear ratio). */
    private static final double TURRET_GEAR_RATIO = 30.0;

    /**
     * Calibration offset in motor rotations.
     * Zero position of the motor encoder != forward-facing turret.
     * Tune this on the real robot.
     */
    private static final double TURRET_CALIBRATION_OFFSET_ROTATIONS = -0.63;

    /** Turret travel limits in degrees. */
    private static final double TURRET_MIN_DEG = -180.0;
    private static final double TURRET_MAX_DEG = 180.0;

    // -----------------------------------------------------------------------
    // Mechanism offsets from robot center
    // -----------------------------------------------------------------------

    private static final Transform2d ROBOT_TO_TURRET =
            new Transform2d(0.148339, 0.127487, Rotation2d.fromDegrees(0));

    private static final Transform2d ROBOT_TO_SHOOTER =
            new Transform2d(0.148339, -0.127487, Rotation2d.fromDegrees(0));

    // -----------------------------------------------------------------------
    // Solver config
    // -----------------------------------------------------------------------

    /** How many times to iterate the TOF->virtual-target->distance loop. 3 is plenty. */
    private static final int SOLVER_ITERATIONS = 3;

    /** Step size in degrees when scanning the real hood range for the best angle. */
    private static final double HOOD_SCAN_STEP_DEG = 0.5;

    /**
     * Efficiency factor [0..1].
     * A ball is not a point mass — drag, spin, and compression loss mean
     * the ball exits with less effective velocity than the flywheel surface speed.
     * Start at 0.85 and tune up/down until stationary shots at known distances are consistent.
     */
    private static final double BALL_EFFICIENCY = 0.85;

    // -----------------------------------------------------------------------
    // Result record
    // -----------------------------------------------------------------------

    /**
     * Everything the three subsystems need to act on.
     * All values are already in the native unit of each subsystem.
     */
    public record AimSolution(
            /** Motor rotations to send to MotionMagic on the turret TalonFX. */
            double turretMotorRotations,

            /** Ticks to send to MotionMagic on the hood TalonFX. */
            double hoodTicks,

            /**
             * RPS to send to the shooter flywheel.
             * Sign is already corrected for your Clockwise_Positive inversion
             * (will be negative for a forward shot).
             */
            double shooterRPS,

            /** Straight-line distance shooter->virtual target (meters). For logging. */
            double distanceMeters,

            /** The lead-corrected point in field space we are aiming at. For logging. */
            Translation2d virtualTarget,

            /** Hood angle in degrees [HOOD_MIN_DEG .. HOOD_MAX_DEG]. For logging/verification. */
            double hoodAngleDeg,

            /** Required exit velocity in m/s. For logging/verification. */
            double exitVelocityMps,

            /** True when the robot is within range and physics produced a valid solution. */
            boolean feasible) {
    }

    // -----------------------------------------------------------------------
    // Public API
    // -----------------------------------------------------------------------

    /**
     * Solve for the complete aim solution given the current drive state.
     * Call this ONCE per periodic loop from your aiming command.
     *
     * @param drive the drive subsystem (for pose + field-relative speeds)
     * @return a fully computed AimSolution (check feasible before firing)
     */
    public AimSolution solve(Drive drive) {
        return solveForTarget(drive, getHubTarget());
    }

    /**
     * Solve for a specific target (hub, pass point, etc.).
     */
    public AimSolution solveForTarget(Drive drive, Translation2d staticTarget) {

        Pose2d robotPose       = drive.getPose();
        ChassisSpeeds fieldSpd = drive.getFieldSpeeds();

        // Compute field positions of the shooter and turret pivots
        Pose2d shooterField = toFieldPose(robotPose, ROBOT_TO_SHOOTER);
        Pose2d turretField  = toFieldPose(robotPose, ROBOT_TO_TURRET);

        // ---- Seed values for the iteration ----
        // Start with the real static target so iteration 0 is a valid static shot.
        Translation2d virtualTarget = staticTarget;
        double tof    = 0.3; // seconds — seed, will converge quickly
        double hoodDeg   = HOOD_MIN_DEG;
        double exitVelMps = 10.0;
        boolean feasible = true;

        for (int iter = 0; iter < SOLVER_ITERATIONS; iter++) {

            // Step 1: straight-line horizontal distance shooter pivot -> virtual target
            double dist = shooterField.getTranslation().getDistance(virtualTarget);

            // Step 2: physics — what hood angle do we need for this distance + height?
            double[] physics = solveHoodAndVelocity(dist, DELTA_HEIGHT_M);

            hoodDeg    = physics[0];
            exitVelMps = physics[1];
            feasible   = physics[2] > 0.5; // flag set to 1.0 if valid

            // Step 3: recompute TOF with these values
            // Horizontal component of exit velocity carries the ball to the target.
            double hoodRad       = Math.toRadians(hoodDeg);
            double horizontalVel = exitVelMps * Math.cos(hoodRad);
            tof = horizontalVel > 0.01 ? dist / horizontalVel : 0.3;

            // Step 4: lead the static target by robot field-relative velocity * TOF
            // (robot moving "toward" the target in field-frame means the ball needs
            //  less lead, so we subtract robot velocity from the target position)
            virtualTarget = leadTarget(staticTarget, fieldSpd, tof);
        }

        // ---- Final distance used for all output values ----
        double finalDist = shooterField.getTranslation().getDistance(virtualTarget);

        // ---- Turret angle ----
        double turretDeg = computeTurretAngle(turretField, robotPose, virtualTarget);
        double turretMotorRot = degreesToMotorRotations(turretDeg) + TURRET_CALIBRATION_OFFSET_ROTATIONS;

        // Clamp to physical limits
        double turretMotorMax = degreesToMotorRotations(TURRET_MAX_DEG);
        double turretMotorMin = degreesToMotorRotations(TURRET_MIN_DEG);
        turretMotorRot = clamp(turretMotorRot, turretMotorMin, turretMotorMax);

        // ---- Hood ticks (offset from real measured minimum angle, clamp to physical limits) ----
        double hoodDegClamped = clamp(hoodDeg, HOOD_MIN_DEG, HOOD_HARD_MAX_DEG);
        double hoodTicks = clamp(
                (hoodDegClamped - HOOD_ANGLE_AT_MIN_TICKS_DEG) * HOOD_TICKS_PER_DEGREE,
                HOOD_MIN_TICKS, HOOD_MAX_TICKS);

        // ---- Flywheel RPS (negative = forward for your motor inversion) ----
        //  exitVelMps = |rps| * 2π * r * efficiency
        //  |rps| = exitVelMps / (2π * r * efficiency)
        double absRPS = exitVelMps / (2.0 * Math.PI * FLYWHEEL_RADIUS_M * BALL_EFFICIENCY);
        absRPS = clamp(absRPS, SHOOTER_MIN_RPS, SHOOTER_MAX_RPS);
        double shooterRPS = -absRPS; // negative for your Clockwise_Positive inversion

        // ---- Mark infeasible if anything is saturated ----
        if (absRPS >= SHOOTER_MAX_RPS || hoodDeg >= HOOD_HARD_MAX_DEG || hoodDeg <= HOOD_MIN_DEG || exitVelMps <= 0.0) {
            feasible = false;
        }

        // ---- Log everything so you can tune ----
        Logger.recordOutput("AimingCalc/VirtualTarget",    virtualTarget);
        Logger.recordOutput("AimingCalc/Distance",         finalDist);
        Logger.recordOutput("AimingCalc/HoodAngleDeg",     hoodDeg);
        Logger.recordOutput("AimingCalc/ExitVelMps",       exitVelMps);
        Logger.recordOutput("AimingCalc/ShooterAbsRPS",    absRPS);
        Logger.recordOutput("AimingCalc/TurretDeg",        turretDeg);
        Logger.recordOutput("AimingCalc/TurretMotorRot",   turretMotorRot);
        Logger.recordOutput("AimingCalc/TOF",              tof);
        Logger.recordOutput("AimingCalc/Feasible",         feasible);

        // ---- DEBUG: also push to SmartDashboard (visible without a log viewer) ----
        // Remove these once everything is confirmed working — they run every loop.
        SmartDashboard.putNumber("DBG/RobotPoseX",      robotPose.getX());
        SmartDashboard.putNumber("DBG/RobotPoseY",      robotPose.getY());
        SmartDashboard.putNumber("DBG/Distance",        finalDist);
        SmartDashboard.putNumber("DBG/HoodAngleDeg",    hoodDeg);
        SmartDashboard.putNumber("DBG/ExitVelMps",      exitVelMps);
        SmartDashboard.putNumber("DBG/ShooterAbsRPS",   absRPS);
        SmartDashboard.putNumber("DBG/TurretDeg",       turretDeg);
        SmartDashboard.putNumber("DBG/TurretMotorRot",  turretMotorRot);
        SmartDashboard.putBoolean("DBG/Feasible",       feasible);

        return new AimSolution(
                turretMotorRot,
                hoodTicks,
                shooterRPS,
                finalDist,
                virtualTarget,
                hoodDeg,
                exitVelMps,
                feasible);
    }

    // -----------------------------------------------------------------------
    // Core physics solver
    // -----------------------------------------------------------------------

    /**
     * Given horizontal distance and vertical rise to the target, find the hood
     * angle (within the REAL physical range of this mechanism) and exit velocity
     * that hits the target, preferring the angle that needs the LEAST velocity.
     *
     * This hood only travels HOOD_MIN_DEG..HOOD_HARD_MAX_DEG (a steep, near-vertical
     * range). The textbook "unconstrained minimum-energy angle" formula
     * (atan(y/x + sqrt(1+(y/x)^2))) computes an angle this mechanism can't reach
     * at typical match distances, so instead we directly scan the real range and
     * solve for velocity at each candidate, keeping whichever needs the least RPS.
     *
     * Projectile equations (no drag — add a drag coefficient later if you want
     * more precision at long range):
     *   horizontal: x = v*cos(θ)*t
     *   vertical:   y = v*sin(θ)*t - ½*g*t²
     * Eliminating t:
     *   v² = g*x² / (cos²(θ)*(2*x*tan(θ) - 2*y))
     *
     * @param x horizontal distance (m)
     * @param y vertical rise, positive = up (m)
     * @return [hoodDeg, exitVelMps, feasibleFlag (1.0 = ok, 0.0 = no solution)]
     */
    private static double[] solveHoodAndVelocity(double x, double y) {

        if (x < 0.1) {
            // Too close — degenerate geometry, hold at minimum angle
            return new double[] {HOOD_MIN_DEG, 0.0, 0.0};
        }

        double bestDeg = -1;
        double bestVel = Double.MAX_VALUE;
        boolean found = false;

        // Scan the real achievable hood range in fine steps and keep the
        // candidate that requires the lowest exit velocity (= lowest RPS = most margin).
        for (double deg = HOOD_MIN_DEG; deg <= HOOD_HARD_MAX_DEG; deg += HOOD_SCAN_STEP_DEG) {
            double rad  = Math.toRadians(deg);
            double cosH = Math.cos(rad);
            double tanH = Math.tan(rad);
            double denom = cosH * cosH * (2.0 * x * tanH - 2.0 * y);

            if (denom <= 0.0) {
                continue; // not a valid trajectory at this angle for this distance
            }

            double vSquared = G * x * x / denom;
            double vel = Math.sqrt(vSquared);

            if (vel < bestVel) {
                bestVel = vel;
                bestDeg = deg;
                found = true;
            }
        }

        if (!found) {
            // No angle in the real range can reach this distance — truly out of range
            return new double[] {HOOD_MAX_DEG, 0.0, 0.0};
        }

        return new double[] {bestDeg, bestVel, 1.0};
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    /**
     * Compute the virtual target position that accounts for robot motion.
     *
     * Insight: the robot is moving in field frame.  The ball, once launched,
     * travels in an inertial frame.  So if the robot has field-velocity V,
     * the ball also inherits V at launch.  That means from the target's
     * perspective, the ball arrives shifted by V*TOF from where the robot
     * was at launch.  To compensate we aim at (target - V*TOF), which in
     * field frame is where the "virtual" target appears to be.
     */
    private static Translation2d leadTarget(
            Translation2d target, ChassisSpeeds fieldSpeeds, double tofSec) {
        return new Translation2d(
                target.getX() - fieldSpeeds.vxMetersPerSecond * tofSec,
                target.getY() - fieldSpeeds.vyMetersPerSecond * tofSec);
    }

    /**
     * Compute turret angle in degrees (robot-relative) to aim at virtualTarget.
     * Uses the turret pivot position, not the robot center.
     */
    private static double computeTurretAngle(
            Pose2d turretFieldPose, Pose2d robotPose, Translation2d virtualTarget) {

        double dx = virtualTarget.getX() - turretFieldPose.getX();
        double dy = virtualTarget.getY() - turretFieldPose.getY();

        double fieldAngleDeg  = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = robotPose.getRotation().getDegrees();

        return normalizeAngle(fieldAngleDeg - robotHeadingDeg);
    }

    /**
     * Transform a robot-relative offset into a field-frame Pose2d.
     * Uses the standard 2D rotation matrix:
     *   x_field = x_local*cos(θ) - y_local*sin(θ) + robot_x
     *   y_field = x_local*sin(θ) + y_local*cos(θ) + robot_y
     */
    private static Pose2d toFieldPose(Pose2d robotPose, Transform2d robotToMechanism) {
        double xL  = robotToMechanism.getX();
        double yL  = robotToMechanism.getY();
        double cos = robotPose.getRotation().getCos();
        double sin = robotPose.getRotation().getSin();
        return new Pose2d(
                xL * cos - yL * sin + robotPose.getX(),
                xL * sin + yL * cos + robotPose.getY(),
                robotPose.getRotation());
    }

    private static Translation2d getHubTarget() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return alliance == Alliance.Blue ? FieldTargets.BLUE_HUB : FieldTargets.RED_HUB;
    }

    private static double degreesToMotorRotations(double degrees) {
        return (degrees / 360.0) * TURRET_GEAR_RATIO;
    }

    public static double normalizeAngle(double angle) {
        while (angle > 180)  angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}