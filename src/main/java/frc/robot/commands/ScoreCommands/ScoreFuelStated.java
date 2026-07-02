package frc.robot.commands.ScoreCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.commands.FeedingCommands.Feed;
import frc.robot.commands.FeedingCommands.FeedStop;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;


public class ScoreFuelStated extends Command {

  private final Drive drive;
  private final Shooter shooter;
  private final Hood hood;

  public 
  ScoreFuelStated(Drive drive, Shooter shooter, Hood hood) {    
    this.drive = drive;
    this.shooter = shooter;
    this.hood = hood;
   addRequirements(shooter);
  }

  @Override public void initialize() {
  }

  @Override public void execute() {
    shooter.shootAtVelocity(shooter.getShooterState());
  }

  @Override public void end(boolean interrupted) {
  }

  @Override public boolean isFinished() { 
    return false; 
}   
}