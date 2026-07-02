package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AimingCalculator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TurretNew;
import frc.robot.subsystems.drive.Drive;

/**
 * DEBUG VERSION — wraps everything in try/catch and prints to the
 * RioLog / Driver Station console so we can see exactly where it dies.
 *
 * Once we find the bug, strip the try/catch back out — it's only here
 * to surface the exception that's currently being swallowed.
 */
public class TrackAndShootCommand extends Command {

    private final Drive drive;
    private final TurretNew turret = TurretNew.getTurretInstance();
    private final Hood hood = Hood.getHoodInstance();
    private final Shooter shooter = Shooter.getShooterInstance();
    private final Feeder feeder = Feeder.getfeederInstance();
    private final AimingCalculator calc = new AimingCalculator();

    private AimingCalculator.AimSolution lastSolution = null;

    public TrackAndShootCommand(Drive drive) {
        this.drive = drive;
        addRequirements(turret, hood, shooter);
        System.out.println("[TrackAndShoot] constructed");
    }

    @Override
    public void initialize() {
        System.out.println("[TrackAndShoot] INITIALIZE — command started");
    }

    @Override
    public void execute() {
        try {
            // ONE solve per loop — all subsystems share this result
            lastSolution = calc.solve(drive);

            System.out.println("[TrackAndShoot] solve() OK -> feasible="
                + lastSolution.feasible()
                + " dist=" + lastSolution.distanceMeters()
                + " hoodDeg=" + lastSolution.hoodAngleDeg()
                + " rps=" + lastSolution.shooterRPS()
                + " turretRot=" + lastSolution.turretMotorRotations());

            turret.applySolution(lastSolution);
            hood.applySolution(lastSolution);
            shooter.applySolution(lastSolution);

            if (shooter.readyToFeed(lastSolution.shooterRPS())) {
                feeder.feederMove(7);
            } else {
                feeder.feederMove(0);
            }

            Logger.recordOutput("TrackAndShoot/Ready",    isReady());
            Logger.recordOutput("TrackAndShoot/Feasible", lastSolution.feasible());

        } catch (Exception e) {
            // THIS is what's currently being silently swallowed.
            // WPILib catches exceptions thrown inside command execute()
            // and just skips that loop iteration — turret/hood never
            // get their applySolution() call, which matches your symptom.
            System.out.println("[TrackAndShoot] *** EXCEPTION IN EXECUTE *** " + e);
            e.printStackTrace();
        }
    }

    public boolean isReady() {
        if (lastSolution == null || !lastSolution.feasible()) return false;
        return turret.atSetpoint(lastSolution)
            && hood.atSetpoint(lastSolution)
            && shooter.atSetpoint(lastSolution);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[TrackAndShoot] END interrupted=" + interrupted);
        shooter.setStopState();
    }
}