package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeOffset extends SequentialCommandGroup {
     
    private final Intake intake;

    public IntakeOffset (){
        this.intake = Intake.getIntakeInstance();
        addRequirements(intake);
        addCommands(new InstantCommand(() -> intake.IntakePose(0)));
        addCommands(new InstantCommand(() -> intake.intakeMove(-0.2)));
        addCommands(new InstantCommand(() -> RobotContainer.blink.setSolidRed()));

    }
}
