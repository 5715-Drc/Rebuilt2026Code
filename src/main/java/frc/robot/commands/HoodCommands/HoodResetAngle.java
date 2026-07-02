package frc.robot.commands.HoodCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Hood;

public class HoodResetAngle extends SequentialCommandGroup {
     
    private final Hood hood;

    public HoodResetAngle (){
        this.hood = Hood.getHoodInstance();
        addRequirements(hood);
        addCommands(new InstantCommand(() -> hood.ResetHoodPose()));
    }
}
