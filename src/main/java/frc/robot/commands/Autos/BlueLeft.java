// package frc.robot.commands.Autos;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.subsystems.Intake;
// import frc.robot.Tunable.Hood;
// import frc.robot.commands.*;
// import frc.robot.commands.IntakeCommands.IntakeOffset;
// import frc.robot.commands.ScoreCommands.ScoreFuel;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.drive.Drive;

// public class BlueLeft extends SequentialCommandGroup {
//   private final Drive drive;
 
 

//   public BlueLeft(Drive drive) {
//     this.drive = drive;
//     addRequirements(drive);

//     try {

//     } catch (Exception e) {
//     }

//     addCommands(
//       new InstantCommand(()-> drive.GoToPose(new Pose2d(7.779, 7.317, new Rotation2d(90)))));

      
//       // new ParallelCommandGroup(
//       //    drive.GoToPose(new Pose2d(7.779, 4.552, Rotation2d.fromDegrees(90))),
//       //    new IntakeIn()


      


    
//   }}