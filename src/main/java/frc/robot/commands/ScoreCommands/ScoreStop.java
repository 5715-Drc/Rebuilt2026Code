package frc.robot.commands.ScoreCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.FeederCommands.FeederStop;
import frc.robot.commands.IndexerCommands.IndexerStop;
import frc.robot.commands.ShooterCommands.ShootStop;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ScoreStop extends SequentialCommandGroup {

    private final Shooter shooter;
    private final Indexer indexer;
    private final Feeder feeder;

    public ScoreStop() {
        this.shooter = Shooter.getShooterInstance();
        this.indexer = Indexer.getIndexerInstance();
        this.feeder = Feeder.getfeederInstance();

        addRequirements(shooter);
        addRequirements(indexer);
        addRequirements(feeder);


        addCommands(new InstantCommand(() -> TunerConstants.setMaxSpeed(5.85)),
                    new IndexerStop(),
                    new FeederStop(),
                    new ShootStop(),
                    new InstantCommand(() -> RobotContainer.blink.setSolidRed()));
    }
    
}
