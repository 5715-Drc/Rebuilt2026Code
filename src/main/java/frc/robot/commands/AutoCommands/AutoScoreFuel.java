package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.ShooterPoint;
import frc.robot.RobotContainer;
import frc.robot.commands.FeedingCommands.Feed;
import frc.robot.commands.FeedingCommands.Pop;
import frc.robot.commands.ScoreCommands.ScoreFuel;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TurretNew;
import frc.robot.subsystems.drive.Drive;


public class AutoScoreFuel extends Command {

  private final Drive drive;
  private final Shooter shooter;
  private final Hood hood;
  private double speed;

  public AutoScoreFuel(Drive drive, Shooter shooter, Hood hood) {    
    this.drive = drive;
    this.shooter = shooter;
    this.hood = hood;
   addRequirements(shooter);
  }

  @Override public void initialize() {
   
  }

 @Override public void execute() {
    double distance = shooter.getShooterDistance(drive, hood); // meters
    ShooterPoint target = Constants.ShooterConstants.interpolate(distance);
    shooter.shootAtVelocity(-target.velocity()+5.0);
    RobotContainer.blink.setHeartBeatBlue();
   

 }

  @Override public void end(boolean interrupted) {
  }

  @Override public boolean isFinished() { 
    return false; 
}   
}