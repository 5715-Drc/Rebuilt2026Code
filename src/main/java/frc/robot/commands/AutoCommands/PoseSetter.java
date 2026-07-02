package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

public class PoseSetter extends SequentialCommandGroup {

    private final Drive drive;

    public PoseSetter(Drive drive) {
        this.drive = drive;

        addCommands(
            new InstantCommand(() -> drive.setPose(new Pose2d(12.136,7.620, Rotation2d.fromDegrees(0))))
            );
    }

}