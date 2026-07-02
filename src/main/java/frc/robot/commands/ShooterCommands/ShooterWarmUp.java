package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class ShooterWarmUp {


  public ShooterWarmUp() {}

  public static Command warmUp(Shooter shooter) {
    return new InstantCommand(() -> shooter.shootAtSpeed(-0.05));
  }



}