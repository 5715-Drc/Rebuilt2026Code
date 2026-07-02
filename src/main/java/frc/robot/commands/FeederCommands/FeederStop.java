package frc.robot.commands.FeederCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Feeder;

public class FeederStop extends SequentialCommandGroup {

    private final Feeder feeder;

    public FeederStop() {
        this.feeder = Feeder.getfeederInstance();
        addRequirements(feeder);
        addCommands(new InstantCommand(() -> feeder.feederstop()));
    }
}
