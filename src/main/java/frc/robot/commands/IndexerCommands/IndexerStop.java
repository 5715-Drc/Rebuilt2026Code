package frc.robot.commands.IndexerCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Indexer;

public class IndexerStop extends SequentialCommandGroup {
     
    private final Indexer indexer;

    public IndexerStop (){
        this.indexer = Indexer.getIndexerInstance();
        addRequirements(indexer);
        addCommands(new InstantCommand(() -> indexer.indexerMove(0.0)));
    }
}
