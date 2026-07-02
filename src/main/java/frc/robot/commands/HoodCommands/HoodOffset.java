package frc.robot.commands.HoodCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

public class HoodOffset extends Command {

  private final Hood hood;

  public HoodOffset(Hood hood) {    
    this.hood = hood;
    addRequirements(hood);
  }

  @Override public void initialize() {}

  @Override public void execute() {
    hood.goToPositionMotionMagic(0);
  }

  @Override public void end(boolean interrupted) {
    
  }

  @Override public boolean isFinished() { 
    return false; 
}   
}