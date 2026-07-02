package frc.robot.commands.IndexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.commands.FeedingCommands.Feed;
import frc.robot.commands.FeedingCommands.FeedStop;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;


public class IndexerStated extends Command {

  private final Indexer indexer;

  public IndexerStated(Indexer indexer) {    
    this.indexer = indexer;
   addRequirements(indexer);
  }

  @Override public void initialize() {
  }

  @Override public void execute() {
    indexer.indexerAtVelocity(indexer.getIndexerState());

    
  }
  @Override public void end(boolean interrupted) {
  }

  @Override public boolean isFinished() { 
    return false; 
}   
}