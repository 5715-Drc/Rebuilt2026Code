package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;

public class Testshooter extends SequentialCommandGroup{
  private final Shooter shooter;

    public Testshooter (){
        this.shooter = Shooter.getShooterInstance();
        addRequirements(shooter);
        addCommands(new InstantCommand(() -> shooter.shootAtVelocity(-40)));
    }

}
