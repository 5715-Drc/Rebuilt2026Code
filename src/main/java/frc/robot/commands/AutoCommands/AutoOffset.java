package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TurretNew;
import frc.robot.subsystems.drive.Drive;


public class AutoOffset extends SequentialCommandGroup {

  private final Drive drive;
  private final TurretNew turret;
  private final Shooter shooter;
  private final Hood hood;

  public AutoOffset(Drive drive, TurretNew turret, Shooter shooter, Hood hood) {    
    this.drive = drive;
    this.turret = turret;
    this.shooter = shooter;
    this.hood = hood;
    addRequirements(turret);
   addCommands(
                new InstantCommand(() -> turret.goToPositionMotionMagic(0)),
                new InstantCommand(() -> hood.goToPositionMotionMagic(0)),
                new InstantCommand(() -> shooter.shootAtSpeed(0))); 
  }
}