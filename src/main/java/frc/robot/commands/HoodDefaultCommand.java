package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
 
public class HoodDefaultCommand extends Command {
 
    private final Hood hood;
 
    // Where to stow the hood when idle.
    // 0 ticks = fully flat (your MIN_TICKS). Safe for driving around.
    private static final double STOW_TICKS = 0.0;
 
    public HoodDefaultCommand(Hood hood) {
        this.hood = hood;
        addRequirements(hood);
    }
 
    @Override
    public void execute() {
        hood.goToPositionMotionMagic(STOW_TICKS);
    }
 
    @Override
    public boolean isFinished() {
        return false;
    }
}