package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretNew;
import frc.robot.subsystems.drive.Drive;


public class turretTrack extends Command {

  private final Drive drive;
  private final TurretNew turret;

  public turretTrack(Drive drive, TurretNew turret) {    
    this.drive = drive;
    this.turret = turret;
    addRequirements(turret);
  }

  @Override public void initialize() {}

  @Override public void execute() {
   turret.aimTurret1(drive);
   RobotContainer.blink.setSolidGreen();
  }

  @Override public void end(boolean interrupted) {
    turret.goToPositionMotionMagic(0);
    RobotContainer.blink.setSolidRed();
  }

  @Override public boolean isFinished() { 
    return false; 
}   
}
// public class LockTurretAtHub extends Command {

//   private final Drive drive;
//   private final Turret turret;
//   private final DoubleSupplier lxSupplier;

//   public LockTurretAtHub(Drive drive, Turret turret, DoubleSupplier lxSupplier) {    
//     this.drive = drive;
//     this.turret = turret;
//     this.lxSupplier = lxSupplier;
//     addRequirements(turret);
//   }

//   @Override
//   public void execute() {
//     double lx = lxSupplier.getAsDouble();
//     turret.goToPositionMotionMagic(
//         turret.getOverComeNAngleToHubTicks(drive, lx)
//     );
//   }

//   @Override
//   public void end(boolean interrupted) {
//     turret.goToPositionMotionMagic(0);
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }