package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.blinkin;


public class IntakeIn extends SequentialCommandGroup {
     
    private final Intake intake;
    private double pose;
    private double speed;

    public IntakeIn (){
        this.intake = Intake.getIntakeInstance();
        addRequirements(intake);
        pose = SmartDashboard.getNumber("intake desired position", -9.5);//-9.63

        addCommands(new InstantCommand(() -> intake.IntakePose(-9.45)));
        // addCommands(new InstantCommand(() -> intake.intakeAtVelocity(-50)));
        addCommands(new InstantCommand(() -> intake.intakeMove(-0.70)));
        // addCommands(new InstantCommand(() -> intake.intakeMove(-1)));
        addCommands(new InstantCommand(() -> RobotContainer.blink.setHeartBeatWhite()));
    }
}
