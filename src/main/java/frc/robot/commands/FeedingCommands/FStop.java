package frc.robot.commands.FeedingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;

public class FStop extends Command {


    private final Feeder feeder;
    private final Indexer indexer;

  public FStop(Feeder feeder , Indexer indexer) {    
    
        this.feeder = Feeder.getfeederInstance();
        this.indexer = Indexer.getIndexerInstance();
    addRequirements(feeder,indexer);
  }

  @Override
   public void initialize() {}

  @Override 
  public void execute() {
    
         feeder.feederMove(0.0);
         indexer.indexerMove(0.0);
  }

  @Override 
  public void end(boolean interrupted) {
    
        feeder.feederMove(0.0);
        indexer.indexerMove(0.0);
  }

  @Override public boolean isFinished() { 
    return true; 
}   
}