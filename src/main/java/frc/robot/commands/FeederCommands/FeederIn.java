package frc.robot.commands.FeederCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Feeder;

public class FeederIn extends SequentialCommandGroup {

    private final Feeder feeder;
    private double speed;

    public FeederIn() {
        this.feeder = Feeder.getfeederInstance();
        addRequirements(feeder);

        speed = SmartDashboard.getNumber("FeederInSpeeed", 0.75);
        addCommands(new InstantCommand(() -> feeder.feederMove(speed)));
    }
}
