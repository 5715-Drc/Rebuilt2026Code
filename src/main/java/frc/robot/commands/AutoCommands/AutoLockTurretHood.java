package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.ShooterPoint;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TurretNew;
import frc.robot.subsystems.drive.Drive;


public class AutoLockTurretHood extends Command {

  private final Drive drive;
  private final TurretNew turret;
  private final Hood hood;
  private final Shooter shooter;

  public AutoLockTurretHood(Drive drive, TurretNew turret, Hood hood, Shooter shooter) {    
    this.drive = drive;
    this.turret = turret;
    this.hood = hood;
    this.shooter = shooter;
    addRequirements(turret, hood);
  }

  @Override public void initialize() {}

  @Override public void execute() {
    double distance = shooter.getShooterDistance(drive, hood); // meters
    ShooterPoint target = Constants.ShooterConstants.interpolate(distance);
    hood.goToPositionMotionMagic(target.hoodPos());

   turret.aimTurret1(drive);
   RobotContainer.blink.setSolidGreen();
  }

  @Override public void end(boolean interrupted) {
    turret.goToPositionMotionMagic(0);
    hood.goToPositionMotionMagic(0);
    RobotContainer.blink.setSolidRed();
  }

  @Override public boolean isFinished() { 
    return false; 
}   
}