package frc.robot.commands.FeedingCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;

public class FeedStop extends SequentialCommandGroup {
     
    private final Feeder feeder;
    private final Indexer indexer;

    public FeedStop (){
        this.feeder = Feeder.getfeederInstance();
        this.indexer = Indexer.getIndexerInstance();
        addRequirements(feeder, indexer);
        addCommands(new InstantCommand(() -> feeder.feedAtVelocity(0.0)));
        addCommands(new InstantCommand(() -> indexer.indexerAtVelocity(0.0)));
    }
}