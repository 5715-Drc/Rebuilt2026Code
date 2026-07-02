package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterV2.ShootingCalculator;
import frc.robot.subsystems.ShooterV2.ShootingCalculator.ShotSolution;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.ShooterV2.ShooterConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TurretNew;

/**
 * ShootWhileMovingCommand
 * ─────────────────────────────────────────────────────────────────────────────
 * A WPILib {@link Command} that:
 *   1. Continuously computes the optimal shot solution from the robot's
 *      current fused pose and velocity (updated every loop ~20 ms).
 *   2. Commands the turret, hood, and flywheel to their calculated setpoints.
 *   3. Fires the feeder/indexer the instant all three are on-target.
 *   4. Works both while the robot is stationary and while it is moving.
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │  SUBSYSTEM INTERFACE ASSUMPTIONS                                        │
 * │  You must implement these methods in your subsystems (stubs shown):     │
 * │                                                                         │
 * │  DriveSubsystem                                                         │
 * │    .getEstimatedPose()   → Pose2d   (fused odometry + vision)           │
 * │    .getFieldRelativeSpeeds() → ChassisSpeeds                            │
 * │                                                                         │
 * │  TurretSubsystem                                                        │
 * │    .setAngle(double deg)                                                │
 * │    .getAngle()           → double  (degrees)                            │
 * │    .isAtTarget()         → boolean                                      │
 * │                                                                         │
 * │  HoodSubsystem                                                          │
 * │    .setAngle(double deg)                                                │
 * │    .getAngle()           → double  (degrees)                            │
 * │    .isAtTarget()         → boolean                                      │
 * │                                                                         │
 * │  ShooterSubsystem                                                       │
 * │    .setVelocityRPS(double rps)                                          │
 * │    .getVelocityRPS()     → double                                       │
 * │    .isAtSpeed()          → boolean                                      │
 * │    .runFeeder()                                                         │
 * │    .stopFeeder()                                                        │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * Usage in RobotContainer:
 * <pre>
 *   new JoystickButton(controller, Button.kRightBumper.value)
 *       .whileTrue(new ShootWhileMovingCommand(drive, turret, hood, shooter));
 * </pre>
 * ─────────────────────────────────────────────────────────────────────────────
 */
public class ShootWhileMovingCommand extends Command {

    // ── Subsystem references ──────────────────────────────────────────────────
    private final Drive drive;
    private final TurretNew turret;
    private final Hood hood;
    private final Shooter shooter;
    private final Feeder feeder;

    // ── State ─────────────────────────────────────────────────────────────────
    private ShotSolution currentSolution;
    private boolean      hasFired;

    /**
     * @param drive    Drive subsystem (provides pose + velocity)
     * @param turret   Turret subsystem
     * @param hood     Hood subsystem
     * @param shooter  Shooter + feeder subsystem
     */
    public ShootWhileMovingCommand(
            Drive drive,
            TurretNew turret,
            Hood hood,
            Shooter shooter
            , Feeder feeder) {

        this.drive   = drive;
        this.turret  = turret;
        this.hood    = hood;
        this.shooter = shooter;
        this.feeder = feeder;

        addRequirements(turret, hood, shooter);
        // Note: we do NOT require the drive subsystem so the driver can still
        // manoeuvre the robot while this command is running.
    }

    // ── Command lifecycle ─────────────────────────────────────────────────────

    @Override
    public void initialize() {
        hasFired        = false;
        currentSolution = null;
        feeder.feederstop();
    }

    @Override
    public void execute() {

        // 1. Grab current robot state
        Pose2d pose   = drive.getPose();
        ChassisSpeeds speeds = drive.getFieldSpeeds();

        // 2. Compute the shot solution for this instant
        currentSolution = ShootingCalculator.calculate(pose, speeds);

        if (!currentSolution.isFeasible) {
            // Can't make this shot — spin up to a safe idle speed and wait
            shooter.shootAtVelocity(20.0);
            feeder.feederstop();
            return;
        }

        // 3. Send setpoints to subsystems
        turret.goToPositionMotionMagic(TurretNew.turretDegreesToMotorRotations(currentSolution.turretAngleDeg));
        hood.goToPositionMotionMagic(hood.DegreesToTicks(currentSolution.hoodAngleDeg));
        shooter.shootAtSpeed(currentSolution.flywheelRPS);

        // 4. Check if ALL three are on-target
        boolean onTarget = ShootingCalculator.isReadyToShoot(
                turret.turretDegreesFromMotorRotations(),
                hood.TicksToDegrees(hood.getHoodPosition()),
                shooter.getSpeed(),
                currentSolution);

        // 5. Fire when ready
        if (onTarget) {
            feeder.feederMove(0.7);
            hasFired = true;
        } else {
            feeder.feederstop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        feeder.feederstop();
        // Leave turret/hood at last position — avoids jerky movement
        // when button is quickly re-pressed. Set to stow if preferred:
        turret.goToPositionMotionMagic(0);
        hood.goToPositionMotionMagic(0);
    }

    @Override
    public boolean isFinished() {
        // This command runs as long as the button is held.
        // It never self-terminates (use .whileTrue() in RobotContainer).
        return false;
    }

    // ── Accessors (for SmartDashboard / Shuffleboard logging) ─────────────────

    /** Returns the most recently computed solution, or null before first execute(). */
    public ShotSolution getCurrentSolution() {
        return currentSolution;
    }

    /** True if the feeder has been triggered at least once this activation. */
    public boolean hasFired() {
        return hasFired;
    }
}