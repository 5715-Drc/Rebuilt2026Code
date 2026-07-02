package frc.robot.commands.ShooterCommands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.ShooterPoint;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TurretNew;
import frc.robot.subsystems.drive.Drive;


public class ShootToDistance extends Command {

  private final Drive drive;
  private final Shooter shooter;
  private final Hood hood;


  public ShootToDistance(Drive drive, Shooter shooter, Hood hood) {    
    this.drive = drive;
    this.shooter = shooter;
    this.hood = hood;
   addRequirements(shooter);
  }

  @Override public void initialize() {}

  @Override public void execute() {
    if(TurretNew.inAllianceZone(drive.getPose())) {
    double distance = shooter.getShooterDistance(drive, hood); // meters
    ShooterPoint target = Constants.ShooterConstants.interpolate(distance);
    shooter.shootAtVelocity(target.velocity());
    }
    else
    {
      shooter.shootAtVelocity(-40);
    }
  }

  @Override public void end(boolean interrupted) {
  }

  @Override public boolean isFinished() { 
    return false; 
}   
}