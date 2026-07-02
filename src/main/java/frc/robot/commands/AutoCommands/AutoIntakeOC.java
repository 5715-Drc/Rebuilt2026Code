package frc.robot.commands.AutoCommands;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants.ShooterPoint;
import frc.robot.subsystems.Intake;
import frc.robot.commands.IntakeCommands.IntakeIn;
import frc.robot.commands.IntakeCommands.IntakeOffset;
import frc.robot.commands.IntakeCommands.IntakeStopAll;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;


public class AutoIntakeOC extends SequentialCommandGroup {

  public final Intake intake;

  public AutoIntakeOC() {    
    this.intake = Intake.getIntakeInstance();
    
    addRequirements(intake);
   addCommands(new IntakeIn());
   new WaitCommand(0.4);
   addCommands(new IntakeOffset());
   new WaitCommand(0.4);
   addCommands(new IntakeIn());
   new WaitCommand(0.4);
   addCommands(new IntakeOffset());
   new WaitCommand(0.4);
   addCommands(new IntakeIn());
   new WaitCommand(0.4);
   addCommands(new IntakeStopAll(intake));

  }

}