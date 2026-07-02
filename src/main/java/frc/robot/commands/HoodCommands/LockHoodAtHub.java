package frc.robot.commands.HoodCommands;

import frc.robot.Constants;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.ShooterPoint;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TurretNew;
import frc.robot.subsystems.drive.Drive;


public class LockHoodAtHub extends Command {

  private final Drive drive;
  private final Hood hood;
  private final Shooter shooter;

  public LockHoodAtHub(Drive drive, Hood hood, Shooter shooter) {    
    this.drive = drive;
    this.hood = hood;
    this.shooter = shooter;
    addRequirements(hood);
  }

  @Override public void initialize() {}

  @Override public void execute() {
    if(TurretNew.inAllianceZone(drive.getPose())) {
    double distance = shooter.getShooterDistance(drive, hood); // meters
     ShooterPoint target = Constants.ShooterConstants.interpolate(distance);
     hood.goToPositionMotionMagic(target.hoodPos());
    
    //double desiredTicks = 0.45071 + (0.334809*(Math.pow(Math.E, 0.427921 * distance)));

   // hood.goToPositionMotionMagic(desiredTicks-0.45);
  }
  else {
    hood.goToPositionMotionMagic(2.3);
  }
  }

  @Override public void end(boolean interrupted) {
    hood.goToPositionMotionMagic(0);
  }

  @Override public boolean isFinished() { 
    return false; 
}   
}