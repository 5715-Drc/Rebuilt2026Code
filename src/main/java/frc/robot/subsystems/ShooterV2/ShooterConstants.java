package frc.robot.subsystems.ShooterV2;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * ShooterConstants
 * ─────────────────────────────────────────────────────────────────────────────
 * All physical, mechanical, and field constants for the shooting system.
 * Tune the values marked with  ← TUNE  before your first match.
 *
 * Unit conventions (strict throughout the codebase):
 *   Distance  → meters
 *   Angle     → degrees (converted to radians internally where needed)
 *   Velocity  → m/s  (robot),  RPS (flywheel)
 *   Mass      → kg
 *   Time      → seconds
 * ─────────────────────────────────────────────────────────────────────────────
 */
public final class ShooterConstants {

    private ShooterConstants() {}   // utility class – no instantiation

    // ─── Ball / Projectile ───────────────────────────────────────────────────

    /** Diameter of the ball in meters. (~15 cm ball) */
    public static final double BALL_DIAMETER_M       = 0.15;

    /** Mass of the ball in kilograms. Adjust after weighing your ball. ← TUNE */
    public static final double BALL_MASS_KG          = 0.215;

    /** Aerodynamic drag coefficient (dimensionless). 0.47 = sphere default. ← TUNE */
    public static final double BALL_DRAG_COEFF       = 0.47;

    /** Cross-sectional area of the ball (m²). Auto-derived from diameter. */
    public static final double BALL_CROSS_SECTION_M2 =
            Math.PI * Math.pow(BALL_DIAMETER_M / 2.0, 2);

    /** Air density at sea level, kg/m³. Adjust for your venue altitude. ← TUNE */
    public static final double AIR_DENSITY_KG_M3     = 1.225;

    // ─── Robot / Shooter Geometry ────────────────────────────────────────────

    /**
     * Height of the ball exit point above the floor, in meters.
     * MEASURE THIS on CompBot — place a tape measure from floor to the
     * flywheel tangent point where the ball leaves.                       ← TUNE
     */
    public static final double SHOOTER_HEIGHT_M      = 0.43;   // placeholder – measure!

    /**
     * Horizontal offset of the shooter pivot from the robot's tracked
     * center (pose origin), in meters.  Positive = forward.              ← TUNE
     */
    public static final double SHOOTER_FORWARD_OFFSET_M = 0.127487;

    /**
     * Lateral offset of the shooter pivot from the robot center, meters.
     * Positive = left.                                                    ← TUNE
     */
    public static final double SHOOTER_LATERAL_OFFSET_M = 0.148339;

    // ─── Turret ──────────────────────────────────────────────────────────────

    /**
     * Turret gear ratio: motor rotations per one full turret rotation.   ← TUNE
     * Example: if it takes 100 motor rotations to spin the turret 360°, set 100.
     */
    public static final double TURRET_GEAR_RATIO     = 30.0;

    /** Turret soft limit – clockwise,  degrees from robot-forward.       ← TUNE */
    public static final double TURRET_MAX_CW_DEG     =  180.0;

    /** Turret soft limit – counter-clockwise, degrees from robot-forward.← TUNE */
    public static final double TURRET_MAX_CCW_DEG    = -180.0;

    /** Turret position tolerance to consider "on target", degrees.       ← TUNE */
    public static final double TURRET_TOLERANCE_DEG  = 1.5;

    // ─── Hood ────────────────────────────────────────────────────────────────

    /**
     * Hood gear ratio: motor rotations per one degree of hood angle.     ← TUNE
     */
    public static final double HOOD_GEAR_RATIO       = 20.0 / 50.0;

    /** Minimum hood angle (flattest / lowest trajectory), degrees.       ← TUNE */
    public static final double HOOD_MIN_DEG          = 90.0 - 25.5;

    /** Maximum hood angle (steepest / highest trajectory), degrees.      ← TUNE */
    public static final double HOOD_MAX_DEG          = 90.0 - 4.0;

    /** Hood position tolerance to consider "on target", degrees.         ← TUNE */
    public static final double HOOD_TOLERANCE_DEG    = 0.5;

    // ─── Flywheel ────────────────────────────────────────────────────────────

    /**
     * Radius of the flywheel in meters.
     * Used to convert between surface speed and RPS.                     ← TUNE
     */
    public static final double FLYWHEEL_RADIUS_M     = 0.051;  // ~2-inch radius

    /**
     * Efficiency factor (0–1): fraction of flywheel surface speed
     * actually transferred to the ball.  Start at 0.85, tune up/down.   ← TUNE
     */
    public static final double FLYWHEEL_EFFICIENCY   = 0.85;

    /** Minimum safe flywheel speed, RPS. */
    public static final double FLYWHEEL_MIN_RPS      = 10.0;

    /** Maximum flywheel speed (motor limit), RPS.                        ← TUNE */
    public static final double FLYWHEEL_MAX_RPS      = 100.0;

    /** Flywheel velocity tolerance to consider "ready to shoot", RPS.    ← TUNE */
    public static final double FLYWHEEL_TOLERANCE_RPS = 1.5;

    // ─── Field / Hub ─────────────────────────────────────────────────────────

    /** Height of the hub opening above the floor, meters. */
    public static final double HUB_HEIGHT_M          = 1.82;

    /**
     * Effective target radius inside the hub.
     * Slightly smaller than physical radius to add a safety margin.      ← TUNE
     */
    public static final double HUB_TARGET_RADIUS_M   = 0.60;

    /**
     * Red alliance hub position (Translation2d in meters, field-relative).
     * Set these to your field's measured hub coordinates.                ← TUNE
     */
    public static final Translation2d RED_HUB_POSITION =
            new Translation2d(11.915, 4.035);   // placeholder – measure your field!

    /**
     * Blue alliance hub position (Translation2d in meters, field-relative).← TUNE
     */
    public static final Translation2d BLUE_HUB_POSITION =
            new Translation2d(4.622, 4.035);    // placeholder – measure your field!

    // ─── Physics ─────────────────────────────────────────────────────────────

    /** Gravitational acceleration, m/s². */
    public static final double GRAVITY_M_S2          = 9.80665;

    /**
     * Physics simulation timestep for the iterative ballistic solver, seconds.
     * Smaller = more accurate but slower. 0.002 s is a good balance.
     */
    public static final double SIM_TIMESTEP_S        = 0.002;

    /**
     * Maximum flight time the solver will iterate over, seconds.
     * A ball should never be in the air longer than this.
     */
    public static final double MAX_FLIGHT_TIME_S     = 3.0;

    // ─── Moving-Shot Prediction ───────────────────────────────────────────────

    /**
     * Total system latency used for lead-target compensation, seconds.
     * Includes: vision pipeline delay + command loop delay + flywheel spin-up.
     * Start at 0.15 s and tune based on observed misses.                 ← TUNE
     */
    public static final double SHOT_LATENCY_S        = 0.15;

    /**
     * Maximum robot speed at which moving-shot compensation is applied, m/s.
     * Above this we wait for the robot to slow down.
     */
    public static final double MAX_MOVING_SHOT_SPEED_M_S = 3.5;
}