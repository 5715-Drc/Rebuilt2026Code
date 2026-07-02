package frc.robot.commands;

// ============================================================
// ShooterDefaultCommand.java
// Idles the shooter when not shooting.
//
// Two options — pick one based on your strategy:
//
// Option A (IDLE_RPS > 0): Keep the flywheel spinning at a low speed.
//   Pro: spins up faster when you press shoot (already moving).
//   Con: burns battery, adds some noise.
//
// Option B (IDLE_RPS = 0): Full stop.
//   Pro: saves battery, quieter.
//   Con: longer spin-up time from dead stop (~0.5-1s depending on your kA).
//
// For a turret robot where you're always tracking, Option A is usually
// worth it — you want to shoot the moment isReady() goes true.
// ============================================================
 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
 
public class ShooterDefaultCommand extends Command {
 
    private final Shooter shooter;
 
    // Set to 0.0 for full stop, or e.g. -20.0 for a warm idle spin
    // (negative because of your Clockwise_Positive inversion)
    private static final double IDLE_RPS = -5.0;
 
    public ShooterDefaultCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }
 
    @Override
    public void execute() {
        shooter.shootAtVelocity(IDLE_RPS);
    }
 
    @Override
    public boolean isFinished() {
        return false;
    }
}
 