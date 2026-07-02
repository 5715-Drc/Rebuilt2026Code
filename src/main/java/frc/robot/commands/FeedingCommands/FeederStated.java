package frc.robot.commands.FeedingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.commands.FeedingCommands.Feed;
import frc.robot.commands.FeedingCommands.FeedStop;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;


public class FeederStated extends Command {

  private final Feeder feeder;

  
 public FeederStated(Feeder feeder) {    
    this.feeder = feeder;
   addRequirements(feeder);
  }

  @Override public void initialize() {
  }

  @Override public void execute() {
    feeder.feedAtVelocity(feeder.getFeederState());

    
  }
  @Override public void end(boolean interrupted) {
  }

  @Override public boolean isFinished() { 
    return false; 
}   
}