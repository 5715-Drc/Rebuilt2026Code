package frc.robot.commands.AutoCommands;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants.ShooterPoint;
import frc.robot.commands.FeedingCommands.Feed;
import frc.robot.commands.ScoreCommands.ScoreStop;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;

public class AutoStopScore extends SequentialCommandGroup {

    private final Shooter shooter;
private final Drive drive;
private final Hood hood;

    public AutoStopScore( Drive drive, Shooter shooter, Hood hood) {
        this.shooter = shooter;
        this.drive = drive;
        this.hood = hood;
        addRequirements(shooter);
    
        addCommands(
             new InstantCommand(() -> shooter.shootAtVelocity(shooter.getTargetVelocity(drive,hood))));
    }
}