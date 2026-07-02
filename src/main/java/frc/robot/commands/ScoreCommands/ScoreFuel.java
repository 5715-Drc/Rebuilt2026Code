package frc.robot.commands.ScoreCommands;

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


public class ScoreFuel extends Command {

  private final Drive drive;
  private final Shooter shooter;
  private final Hood hood;
  private double speed;

  public ScoreFuel(Drive drive, Shooter shooter, Hood hood) {    
    this.drive = drive;
    this.shooter = shooter;
    this.hood = hood;
   addRequirements(shooter);
  }

  @Override public void initialize() {
    TunerConstants.setMaxSpeed(2);
    // speed = SmartDashboard.getNumber("ShooterSpeed", -10);
    // new Pop();
  }

  // @Override public void execute() {
  //   speed = SmartDashboard.getNumber("ShooterSpeed", speed);
  //   double distance = shooter.getShooterDistance(drive, hood); // meters
  //   // ShooterPoint target = Constants.ShooterConstants.interpolate(distance);
  //   // shooter.shootAtVelocity(speed);

  //double desiredVelocity = 39.378 * (Math.pow(Math.E, 0.115 * distance));
  //   shooter.shootAtVelocity(desiredVelocity);

  //   RobotContainer.blink.setHeartBeatBlue();
      
  //   if(shooter.getSpeed() < (speed + 5)) {
  //     CommandScheduler.getInstance().schedule(new Feed());
  //   }
  // }
 @Override public void execute() {
    double distance = shooter.getShooterDistance(drive, hood); // meters
    ShooterPoint target = Constants.ShooterConstants.interpolate(distance);
    shooter.shootAtVelocity(-target.velocity()+2.5);
   // double desiredVelocity = 39.378 * (Math.pow(Math.E, 0.115 * distance));
    RobotContainer.blink.setHeartBeatBlue();
     // double Dvs = -desiredVelocity + 4.5
      ;
       // shooter.shootAtVelocity(Dvs);

    //     if(shooter.getSpeed() < (Dvs+ 5)) {
    //   CommandScheduler.getInstance().schedule(new Feed());
    //}
    if(shooter.getSpeed() < (-target.velocity() + 5)) {
      CommandScheduler.getInstance().schedule(new Feed());
    }

 }

  @Override public void end(boolean interrupted) {
  }

  @Override public boolean isFinished() { 
    return false; 
}   
}