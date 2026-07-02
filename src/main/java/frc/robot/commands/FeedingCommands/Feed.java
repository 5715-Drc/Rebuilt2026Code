package frc.robot.commands.FeedingCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;

public class Feed extends SequentialCommandGroup {
     
    private final Feeder feeder;
    private final Indexer indexer;

     

    public Feed (){
        this.feeder = Feeder.getfeederInstance();
        this.indexer = Indexer.getIndexerInstance();
        addRequirements(feeder, indexer);
        // addCommands(new InstantCommand(() -> feeder.feedAtVelocity(100)));
        addCommands(new InstantCommand(() -> feeder.feederMove(1)));
        new WaitCommand(0.1);
        // addCommands(new InstantCommand(() -> indexer.indexerAtVelocity(-80)));
        addCommands(new InstantCommand(() -> indexer.indexerMove(-1)));

    }
}