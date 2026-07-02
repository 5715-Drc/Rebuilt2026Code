// package frc.robot.commands.TurretCommands;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Robot;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Hood;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Turret;
// import frc.robot.subsystems.blinkin;
// import frc.robot.subsystems.drive.Drive;


// public class LockTurretAtHub extends Command {

//   private final Drive drive;
//   private final Turret turret;
//   private final Shooter shooter;
//   private final Hood hood;

//   public LockTurretAtHub(Drive drive, Turret turret, Shooter shooter, Hood hood) {    
//     this.drive = drive;
//     this.turret = turret;
//     this.shooter = shooter;
//     this.hood = hood;
//     addRequirements(turret);
//   }

//   @Override public void initialize() {}

//   @Override public void execute() {
//    turret.goToPositionMotionMagic(turret.getOverComeNAngleToHubTicks(drive, shooter, hood));
//    RobotContainer.blink.setSolidGreen();
//   }

//   @Override public void end(boolean interrupted) {
//     turret.goToPositionMotionMagic(0);
//     RobotContainer.blink.setSolidRed();
//   }

//   @Override public boolean isFinished() { 
//     return false; 
// }   
// }
// // public class LockTurretAtHub extends Command {

// //   private final Drive drive;
// //   private final Turret turret;
// //   private final DoubleSupplier lxSupplier;

// //   public LockTurretAtHub(Drive drive, Turret turret, DoubleSupplier lxSupplier) {    
// //     this.drive = drive;
// //     this.turret = turret;
// //     this.lxSupplier = lxSupplier;
// //     addRequirements(turret);
// //   }

// //   @Override
// //   public void execute() {
// //     double lx = lxSupplier.getAsDouble();
// //     turret.goToPositionMotionMagic(
// //         turret.getOverComeNAngleToHubTicks(drive, lx)
// //     );
// //   }

// //   @Override
// //   public void end(boolean interrupted) {
// //     turret.goToPositionMotionMagic(0);
// //   }

// //   @Override
// //   public boolean isFinished() {
// //     return false;
// //   }
// // }