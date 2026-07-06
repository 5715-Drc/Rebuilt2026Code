// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IsHubActive;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HoodDefaultCommand;
import frc.robot.commands.ShootWhileMovingCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.commands.TrackAndShootCommand;
import frc.robot.commands.TurretDefaultCommand;
import frc.robot.commands.AutoCommands.AutoIntakeOC;
import frc.robot.commands.AutoCommands.AutoLockTurretHood;
import frc.robot.commands.AutoCommands.AutoOffset;
import frc.robot.commands.AutoCommands.AutoScoreFuel;
import frc.robot.commands.AutoCommands.AutoStopScore;
import frc.robot.commands.AutoCommands.AutoStopScoreFuel;
import frc.robot.commands.AutoCommands.PoseSetter;
import frc.robot.commands.FeederCommands.Feederout;
import frc.robot.commands.FeedingCommands.FStop;
import frc.robot.commands.FeedingCommands.Feed;
import frc.robot.commands.FeedingCommands.FeedStop;
import frc.robot.commands.FeedingCommands.FeederStated;
import frc.robot.commands.HoodCommands.HoodOffset;
import frc.robot.commands.HoodCommands.HoodResetAngle;
import frc.robot.commands.HoodCommands.LockHoodAtHub;
import frc.robot.commands.IndexerCommands.IndexerOut;
import frc.robot.commands.IndexerCommands.IndexerStated;
import frc.robot.commands.IntakeCommands.IntakeIn;
import frc.robot.commands.IntakeCommands.IntakeOffset;
import frc.robot.commands.IntakeCommands.IntakeReverse;
import frc.robot.commands.IntakeCommands.IntakeStopAll;
import frc.robot.commands.ScoreCommands.ScoreFuel;
import frc.robot.commands.ScoreCommands.ScoreFuelStated;
import frc.robot.commands.ScoreCommands.ScoreStop;
import frc.robot.commands.ShooterCommands.ShootStop;
import frc.robot.commands.ShooterCommands.Testshooter;
import frc.robot.commands.TurretCommands.TurretOffset;
import frc.robot.commands.TurretCommands.turretTrack;
import frc.robot.generated.TunerConstants;
//import frc.robot.subsystems.Climb;
// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TurretNew;
import frc.robot.subsystems.blinkin;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Hood hood = Hood.getHoodInstance();
  private final Shooter shooter = Shooter.getShooterInstance();
  private final Feeder feeder = Feeder.getfeederInstance();
  private final Intake intake = Intake.getIntakeInstance();
 private final Indexer indexer = Indexer.getIndexerInstance();
   private final IsHubActive isHubActive = new IsHubActive();

  //private final Climb climb = Climb.getClimbInstance();

 private final TurretNew turretnew = TurretNew.getTurretInstance();

 public static blinkin blink = new blinkin();
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final SendableChooser<Command> autoChooserAuto;

  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
        switch (Constants.currentMode) {

        case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation));
       break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation));
        break;
        }
            autoChooser = new LoggedDashboardChooser<>("Auto Calibrations", AutoBuilder.buildAutoChooser());


          autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));



        //tunable parameters for commands
      SmartDashboard.putNumber("ShooterSpeed", -10);
      SmartDashboard.putNumber("FeederOutSpeeed ",-0.4);
      SmartDashboard.putNumber("FeederInSpeeed ",0.75);
      SmartDashboard.putNumber("indexer speed", -0.6);
      SmartDashboard.putNumber("intake desired position", -9.63);
      SmartDashboard.putBoolean("practice", false);
      
SmartDashboard.putNumber("Turret/SpeedVX", 0);
SmartDashboard.putNumber("Turret/SpeedVY", 0);


   // Set up auto routines
    NamedCommands.registerCommand("PoseSetter", new PoseSetter(drive));
    NamedCommands.registerCommand("IntakeIn", new IntakeIn());
    NamedCommands.registerCommand("IntakeOffset", new IntakeOffset());
    NamedCommands.registerCommand("AutoLookHood", new LockHoodAtHub(drive,hood,shooter));
    NamedCommands.registerCommand("AutoLookTurret", new turretTrack(drive,turretnew));
    NamedCommands.registerCommand("AutoTrackHub", new AutoLockTurretHood(drive,turretnew, hood, shooter));
    // NamedCommands.registerCommand("AutoScoreFuel", new AutoStopScore(drive,shooter, hood));
    NamedCommands.registerCommand("ScoreTrackedEnable", new InstantCommand(() -> shooter.shootAtVelocity(shooter.getTargetVelocity(drive,hood))));
    NamedCommands.registerCommand("AutoIndexerDisable", new InstantCommand(() -> indexer.setStopIndexerState()));
    NamedCommands.registerCommand("AutoFeederDisable",new InstantCommand(()-> feeder.setStopFeederState()));
    NamedCommands.registerCommand("AutoIndexerEnable", new InstantCommand(() -> indexer.getIndexerVelocity()));
    NamedCommands.registerCommand("AutoFeederEnable", new InstantCommand(()-> feeder.getFeederVelocity()));
    NamedCommands.registerCommand("ScoreTrackedDisable", new InstantCommand(() -> shooter.setStopState()));
    NamedCommands.registerCommand("AutoOffset", new AutoOffset(drive, turretnew, shooter, hood));
    NamedCommands.registerCommand("setPose", Commands.runOnce(() -> drive.setPose(new Pose2d(12.136, 7.620, Rotation2d.fromDegrees(0))),drive));
   
    NamedCommands.registerCommand("AutoStopScore" , new AutoStopScoreFuel(drive,shooter,hood));  
    NamedCommands.registerCommand("AutoScoreFuel" , new AutoScoreFuel(drive,shooter,hood));  

    NamedCommands.registerCommand("Autoinatakeoc", new AutoIntakeOC());
    NamedCommands.registerCommand("IntakeStopAll", new IntakeStopAll(intake));
    NamedCommands.registerCommand("Feed", new Feed());
    NamedCommands.registerCommand("FeedStop", new FStop(feeder,indexer));

    autoChooserAuto = AutoBuilder.buildAutoChooser();
   
    SmartDashboard.putData("Auto Chooser", autoChooserAuto);
    autoChooserAuto.setDefaultOption("No Auto", null);
    autoChooserAuto.addOption("RedRight-2Cycles+Outpost", new PathPlannerAuto("RR"));
    autoChooserAuto.addOption("BlueRight-2Cycles+Outpost", new PathPlannerAuto("BR"));
    autoChooserAuto.addOption("PitTest", new PathPlannerAuto("PitTest"));
    autoChooserAuto.addOption("M4", new PathPlannerAuto("M4"));
    autoChooserAuto.addOption("M2", new PathPlannerAuto("M2"));
    autoChooserAuto.addOption("CMDTest", new PathPlannerAuto("CMDTest"));
    autoChooserAuto.addOption("F-RL-AUTO", new PathPlannerAuto("F-RL-AUTO"));
    autoChooserAuto.addOption("F-RM-Auto", new PathPlannerAuto("F-RM-Auto"));
    autoChooserAuto.addOption("F-Rr-AUTO", new PathPlannerAuto("F-RR-AUTO"));
    autoChooserAuto.addOption("F-BR-AUTO", new PathPlannerAuto("F-BR-AUTO"));
    autoChooserAuto.addOption("F-BL-AUTO", new PathPlannerAuto("F-BL-AUTO"));
    autoChooserAuto.addOption("F-BM-AUTO", new PathPlannerAuto("F-BM-Auto"));
    autoChooserAuto.addOption("Copy-RR", new PathPlannerAuto("Copy-of-F-RR-AUTO"));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  
  
  private void configureButtonBindings() {

        TrackAndShootCommand trackAndShoot = new TrackAndShootCommand(drive);

    
  drive.setDefaultCommand(
    DriveCommands.joystickDrive(
                      drive,
                      () -> -controller.getLeftY(),
                      () -> -controller.getLeftX(),
                      () -> -controller.getRightX()));

  shooter.setDefaultCommand(
    new ScoreFuelStated(drive, shooter, hood)
  );  
//feeder.setDefaultCommand(new FeederStated(feeder));
//indexer.setDefaultCommand(new IndexerStated(indexer));
  // turretnew.setDefaultCommand(
  //   new turretTrack(drive, turretnew)
  // );

  // hood.setDefaultCommand(
  //   new LockHoodAtHub(drive, hood, shooter)
  // );
shooter.setDefaultCommand(new ScoreFuelStated(drive, shooter, hood));
hood.setDefaultCommand(new LockHoodAtHub(drive, hood, shooter));
turretnew.setDefaultCommand(new turretTrack(drive, turretnew));

    // shooter.setDefaultCommand(new ShooterDefaultCommand(shooter));
    // hood.setDefaultCommand(new HoodDefaultCommand(hood));  
    // turretnew.setDefaultCommand(new TurretDefaultCommand(turretnew));  

    // Reset gyro to 0° when Start button is pressed
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // //Lock at the hub
    // controller.b().toggleOnTrue(new LockHoodAtHub(drive, hood, shooter));
    // controller.b().toggleOnTrue(new turretTrack(drive, turretnew));
    // controller.b().toggleOnTrue(new HoodOffset(hood));
    // controller.b().toggleOnTrue(new TurretOffset(turretnew));

    //controller.rightBumper().whileTrue(trackAndShoot);

   // controller.a().whileTrue(new InstantCommand(() -> feeder.feederMove(7)).onlyIf(trackAndShoot::isReady));


    //Score fuel
    // controller.rightBumper()
    // .onTrue(new InstantCommand(() -> shooter.setTrackedVelocity(drive, hood))).onTrue(new InstantCommand(() -> TunerConstants.setMaxSpeed(2)))
    // .onFalse(new InstantCommand(() -> shooter.setStopState())).onFalse(new InstantCommand(() -> TunerConstants.setMaxSpeed(5.85)));
    controller.a().toggleOnTrue(new ScoreFuel(drive, shooter, hood)).toggleOnFalse(new ScoreStop());

     controller.rightBumper().onTrue(new IntakeIn()).onFalse(new IntakeOffset());
    //controller.rightBumper().onTrue(new Testshooter()).onFalse(new ShootStop());
    // controller.rightBumper().onTrue(new ShootWhileMovingCommand(drive, turretnew, hood, shooter, feeder));

    //k
    // controller.leftBumper().onTrue(new IntakeIn()).onFalse(new IntakeOffset());
      controller.leftBumper().toggleOnTrue(new ScoreFuel(drive, shooter, hood)).toggleOnFalse(new ScoreStop());
   

    //intake control
     controller.pov(90).onTrue(new InstantCommand(() -> intake.intakeRotation(0.4))).onFalse(new InstantCommand(() -> intake.intakeRotation(0)));
     controller.pov(270).onTrue(new InstantCommand(() -> intake.intakeRotation(-0.6))).onFalse(new InstantCommand(() -> intake.intakeRotation(0)));
// Hood control (Using whileTrue + StartEndCommand)

// Hood Manual Move Up (POV 0)
controller.pov(0).whileTrue(
    new StartEndCommand(
        () -> hood.HoodMove(0.4), 
        () -> hood.lockCurrentPosition(), // Tells hood: "Hold exactly where I left you"
        hood
    )
);

// Hood Manual Move Down (POV 180)
controller.pov(180).whileTrue(
    new StartEndCommand(
        () -> hood.HoodMove(-0.4), 
        () -> hood.lockCurrentPosition(), 
        hood
    )
);

    //Indexer
    controller.b()
    .onTrue(new InstantCommand(() -> indexer.indexerMove(0.4))).onTrue(new InstantCommand(() -> feeder.feederMove(-0.5)))
    .onFalse(new InstantCommand(() -> feeder.feederMove(0.0))).onFalse(new InstantCommand(() -> indexer.indexerMove(0)));

   controller.x().onTrue(new IntakeReverse()).onFalse(new IntakeOffset());

    controller.back().onTrue(new InstantCommand(() -> intake.ResetIntakePose()));
   
   //controller.pov(90).onTrue(new InstantCommand(() -> intake.intakeMove(0.4))).onFalse(new InstantCommand(() -> intake.intakeMove(0)));

    //Climber
    // controller.y().onTrue(new InstantCommand(()-> climb.ClimbMove(-0.4))).onFalse(new InstantCommand(()-> climb.ClimbMove(0)));
    // controller.a().onTrue(new InstantCommand(()-> climb.ClimbMove(0.4))).onFalse(new InstantCommand(()-> climb.ClimbMove(0)));

    //  controller.y().onTrue(new InstantCommand(() -> hood.goToPositionMotionMagic(2))); //hood2
    //  controller.a().onTrue(new InstantCommand(() -> hood.goToPositionMotionMagic(0))); //hood Offset
                    
    // //Feeder + Indexer control
    // controller.y().onTrue(new Feed()).onFalse(new FeedStop());
    // controller.a().onTrue(new Pop()).onFalse(new FeedStop());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   return autoChooserAuto.getSelected();
  }


}
