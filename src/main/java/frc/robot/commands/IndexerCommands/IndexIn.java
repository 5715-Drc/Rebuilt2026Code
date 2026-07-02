package frc.robot.commands.IndexerCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Indexer;

public class IndexIn extends SequentialCommandGroup {
     
    private final Indexer indexer;
    private double speed;

    public IndexIn (){
        this.indexer = Indexer.getIndexerInstance();
        addRequirements(indexer);

        speed = SmartDashboard.getNumber("indexer speed", -0.6);
        addCommands(new InstantCommand(() -> indexer.indexerMove(speed)));
    }
}
