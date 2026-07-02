package frc.robot.commands.AutoCommands;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants.ShooterPoint;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;


public class AutoLookHood extends SequentialCommandGroup {

  private final Drive drive;
  private final Hood hood;
  private final Shooter shooter;

  public AutoLookHood(Drive drive, Hood hood, Shooter shooter) {    
    this.drive = drive;
    this.hood = hood;
    this.shooter = shooter;
    addRequirements(hood);
    double distance = shooter.getShooterDistance(drive, hood); // meters
     ShooterPoint target = Constants.ShooterConstants.interpolate(distance);
    addCommands(new InstantCommand(() -> hood.goToPositionMotionMagic(target.hoodPos())));
   
  }

}