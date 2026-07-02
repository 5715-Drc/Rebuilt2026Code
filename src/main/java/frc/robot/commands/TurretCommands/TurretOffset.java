package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretNew;

public class TurretOffset extends Command {

  private final TurretNew turret;

  public TurretOffset(TurretNew turret) {    
    this.turret = turret;
    addRequirements(turret);
  }

  @Override public void initialize() {}

  @Override public void execute() {
   turret.goToPositionMotionMagic(0);
  }

  @Override public void end(boolean interrupted) {
  }

  @Override public boolean isFinished() { 
    return false; 
}   
}