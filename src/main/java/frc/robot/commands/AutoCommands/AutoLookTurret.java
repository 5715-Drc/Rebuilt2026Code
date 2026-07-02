package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TurretNew;
import frc.robot.subsystems.drive.Drive;


public class AutoLookTurret extends SequentialCommandGroup {

  private final Drive drive;
  private final TurretNew turret;

  public AutoLookTurret(Drive drive, TurretNew turret) {    
    this.drive = drive;
    this.turret = turret;

    addRequirements(turret);
   addCommands(new InstantCommand(()-> turret.aimTurret1(drive)
)); 
  }
  
}