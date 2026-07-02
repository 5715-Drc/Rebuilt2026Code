package frc.robot.commands;
 
// ============================================================
// TurretDefaultCommand.java
// Holds the turret at its current position when not aiming.
// Prevents the turret from going limp and drifting.
// ============================================================
 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretNew;
 
public class TurretDefaultCommand extends Command {
 
    private final TurretNew turret;
    private double heldPosition = 0.0;
 
    public TurretDefaultCommand(TurretNew turret) {
        this.turret = turret;
        addRequirements(turret);
    }
 
    @Override
    public void initialize() {
        // Snapshot position the moment this becomes the active command
        // so we hold exactly where we stopped, not some stale value
        heldPosition = turret.getTurretPos();
    }
 
    @Override
    public void execute() {
        turret.goToPositionMotionMagic(heldPosition);
    }
 
    @Override
    public boolean isFinished() {
        return false; // runs until interrupted by TrackAndShootCommand
    }
}