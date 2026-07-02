package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;

public class ShootStop extends SequentialCommandGroup {
     
    private final Shooter shooter;

    public ShootStop (){
        this.shooter = Shooter.getShooterInstance();
        addRequirements(shooter);
        addCommands(new InstantCommand(() -> shooter.stopShooting()));
    }
}
