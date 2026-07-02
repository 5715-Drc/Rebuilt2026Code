package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.ShooterPoint;
import frc.robot.RobotContainer;
import frc.robot.commands.FeedingCommands.Feed;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;

public class AutoScoreFuel extends Command {

  private final Drive drive;
  private final Shooter shooter;
  private final Hood hood;
  

  public AutoScoreFuel(Drive drive, Shooter shooter, Hood hood) {    
    this.drive = drive;
    this.shooter = shooter;
    this.hood = hood;
    
    addRequirements(shooter);
  }

  @Override 
  public void initialize() {
  }

  @Override 
  public void execute() {
    double distance = shooter.getShooterDistance(drive, hood);
    ShooterPoint target = Constants.ShooterConstants.interpolate(distance);
    double targetVel = -target.velocity() + 4.5;
    
    shooter.shootAtVelocity(targetVel);
    
  }

  @Override 
  public void end(boolean interrupted) {
  }

  @Override 
  public boolean isFinished() { 
    return false; 
  }   
}