package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TurretNew;

public class IntakeStopAll extends Command {

  private final Intake intake;

  public IntakeStopAll(Intake intake) {    
    this.intake = intake;
    addRequirements(intake);
  }

  @Override public void initialize() {}

  @Override public void execute() {
   intake.intakeMove(0);
  }

  @Override public void end(boolean interrupted) {
       intake.intakeMove(0);

  }

  @Override public boolean isFinished() { 
    return true; 
}   
}