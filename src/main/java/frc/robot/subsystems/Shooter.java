package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;


import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Tunable;
import frc.robot.Constants.ShooterConstants.ShooterPoint;
import frc.robot.subsystems.drive.Drive;

public class Shooter extends SubsystemBase {
   
  private static Shooter shooterInstance = null;
  private TalonFX shooterMotor;
  private TalonFX shooterMotor1;
  private DutyCycleOut dutyCycleOutForMove;
  private TalonFXConfiguration shooterConfiguration;
  private TalonFXConfiguration shooterConfiguration1;
  private MotionMagicConfigs motionMagicConfiguration;
  final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

  private double desiredState = 0;


  private Transform2d RobotToShooter = new Transform2d(0.148339, -0.127487, Rotation2d.fromDegrees(0));
  private Pose2d blueHubPose = new Pose2d(4.626, 4.035, new Rotation2d());
  private Pose2d redHubPose = new Pose2d(11.9, 4.035, new Rotation2d());

  private final TrackingHelpers trackingHelpers = new TrackingHelpers();



  private Shooter() {
    
    shooterMotor = new TalonFX(33);
    shooterMotor1 = new TalonFX(34);

    shooterConfiguration1 = new TalonFXConfiguration();
    shooterConfiguration = new TalonFXConfiguration();  

    applyTunableConfiguration();
    

    shooterMotor1.setControl(new Follower(shooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    shooterMotor.getConfigurator().apply(shooterConfiguration);
    shooterMotor1.getConfigurator().apply(shooterConfiguration);
    dutyCycleOutForMove = new DutyCycleOut(0);
  }
 private void applyTunableConfiguration() {
    // Current limits
    shooterConfiguration.CurrentLimits.SupplyCurrentLimit = Tunable.Shooter.LsupplyCurrentLimit.get(); 
    shooterConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterConfiguration.CurrentLimits.SupplyCurrentLowerLimit = Tunable.Shooter.LsupplyCurrentLowerLimit.get();
    shooterConfiguration.CurrentLimits.SupplyCurrentLowerTime = Tunable.Shooter.LsupplyCurrentLowerTime.get();
    
    // PID Slot0
    shooterConfiguration.Slot0.kP = Tunable.Shooter.LkP.get();
    shooterConfiguration.Slot0.kI = Tunable.Shooter.LkI.get();
    shooterConfiguration.Slot0.kD = Tunable.Shooter.LkD.get();
    shooterConfiguration.Slot0.kS = Tunable.Shooter.LkS.get();
    shooterConfiguration.Slot0.kV = Tunable.Shooter.LkV.get();
    shooterConfiguration.Slot0.kA = Tunable.Shooter.LkA.get();

    // Motor configuration
    shooterConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Motion Magic
    motionMagicConfiguration = shooterConfiguration.MotionMagic;
    motionMagicConfiguration.MotionMagicCruiseVelocity = Tunable.Shooter.LmotionMagicCruiseVelocity.get();
    motionMagicConfiguration.MotionMagicAcceleration = Tunable.Shooter.LmotionMagicAcceleration.get();
    motionMagicConfiguration.MotionMagicJerk = Tunable.Shooter.LmotionMagicJerk.get();
    
    // Apply configuration to motors
    shooterMotor.getConfigurator().apply(shooterConfiguration);
    shooterMotor1.getConfigurator().apply(shooterConfiguration);
}
  //This method calculates the position of the hood relative to the field 
  //It uses the position of the hood relative to the robot relative to the field
  public Pose2d 
  shooterToField(Drive drive){
    double xlocal = RobotToShooter.getX();
    double ylocal = RobotToShooter.getY();
    double xGlobal = xlocal * Math.cos(drive.getRotation().getRadians()) + ylocal * Math.sin(drive.getRotation().getRadians()) + drive.getPose().getX();
    double yGlobal = ylocal * Math.cos(drive.getRotation().getRadians()) + xlocal * Math.sin(drive.getRotation().getRadians()) + drive.getPose().getY();

    return new Pose2d(xGlobal, yGlobal, drive.getRotation());
  }

  public static Shooter getShooterInstance() {
    if (shooterInstance == null) shooterInstance = new Shooter();
    return shooterInstance;
  }

  public void setTrackedVelocity(Drive drive, Hood hood) {
     double distance = this.getShooterDistance(drive, hood); // meters
     ShooterPoint target = Constants.ShooterConstants.interpolate(distance);
     desiredState = target.velocity();

   // desiredState = (-0.562542*(Math.pow(Math.E, 0.78493 * getShooterDistance(drive, hood)))) - 48.;
  }

  public double getTargetVelocity(Drive drive, Hood hood) {
    double distance = this.getShooterDistance(drive, hood); // meters
    ShooterPoint target = Constants.ShooterConstants.interpolate(distance);
    return -target.velocity() + 1.5;
    // double targetVel = (-0.562542*(Math.pow(Math.E, 0.78493 * getShooterDistance(drive, hood)))) - 46.;
    // return targetVel;
  }

  public void setStopState() {
    desiredState = -5;
  }
  
  public double getShooterState() {
    return desiredState;
  }

  public double calculateAdjustedShooterVelocity(Drive drive, edu.wpi.first.math.geometry.Translation2d targetTranslation) {
      edu.wpi.first.math.geometry.Pose2d robotPose = drive.getPose();
      edu.wpi.first.math.kinematics.ChassisSpeeds speeds = drive.getFieldSpeeds();

      edu.wpi.first.math.geometry.Translation2d robotToTarget = targetTranslation.minus(robotPose.getTranslation());
      double distance = robotToTarget.getNorm();
      edu.wpi.first.math.geometry.Translation2d unitVector = robotToTarget.div(distance);

      double baseShooterVelocityMps = Constants.ShooterConstants.interpolate(distance).velocity(); 

      double robotVelocityTowardsTarget = (speeds.vxMetersPerSecond * unitVector.getX()) 
                                        + (speeds.vyMetersPerSecond * unitVector.getY());

      double adjustedBallVelocityMps = baseShooterVelocityMps - robotVelocityTowardsTarget;

      return convertMpsToRps(adjustedBallVelocityMps);
  }

  private double convertMpsToRps(double mps) {
    double wheelDiameterMeters = 0.1016; 
    double gearRatio = 1.0; 
    double wheelCircumference = Math.PI * wheelDiameterMeters;
    double motorRps = (mps / wheelCircumference) * gearRatio;

    return motorRps;
}

    public double getShooterDistance(Drive drive, Hood hood) {
    Pose2d shooterPose = shooterToField(drive);

    if(DriverStation.getAlliance().get() == Alliance.Blue) {
        if(drive.getChassisSpeeds().vxMetersPerSecond > 0.1 || drive.getChassisSpeeds().vxMetersPerSecond < -0.1 || drive.getChassisSpeeds().vyMetersPerSecond > 0.1 || drive.getChassisSpeeds().vyMetersPerSecond < -0.1){
          Logger.recordOutput("RealOutputs/ShooterDistance", shooterPose.getTranslation().getDistance(trackingHelpers.predictTargetPos(blueHubPose.getTranslation(), drive.getFieldSpeeds(), Seconds.of(0.5)/*trackingHelpers.calculateTimeOfFlight(getExitLinearVelocity(), Degrees.of(hood.getHoodAngleDegrees(drive, shooterInstance)), getAbsoluteDistance(drive))*/)));
          return shooterPose.getTranslation().getDistance(trackingHelpers.predictTargetPos(blueHubPose.getTranslation(), drive.getFieldSpeeds(), Seconds.of(0.5)/*trackingHelpers.calculateTimeOfFlight(getExitLinearVelocity(), Degrees.of(hood.getHoodAngleDegrees(drive, shooterInstance)), getAbsoluteDistance(drive))*/));
        }
        else
          Logger.recordOutput("RealOutputs/ShooterDistance", shooterPose.getTranslation().getDistance(blueHubPose.getTranslation()));
          return shooterPose.getTranslation().getDistance(blueHubPose.getTranslation());
      }
    if (DriverStation.getAlliance().get() == Alliance.Red){
        if(drive.getChassisSpeeds().vxMetersPerSecond > 0.1 || drive.getChassisSpeeds().vxMetersPerSecond < -0.1 || drive.getChassisSpeeds().vyMetersPerSecond > 0.1 || drive.getChassisSpeeds().vyMetersPerSecond < -0.1){
          Logger.recordOutput("RealOutputs/ShooterDistance", shooterPose.getTranslation().getDistance(trackingHelpers.predictTargetPos(redHubPose.getTranslation(), drive.getFieldSpeeds(), Seconds.of(0.5)/*trackingHelpers.calculateTimeOfFlight(getExitLinearVelocity(), Degrees.of(hood.getHoodAngleDegrees(drive, shooterInstance)), getAbsoluteDistance(drive))*/)));
          return shooterPose.getTranslation().getDistance(trackingHelpers.predictTargetPos(redHubPose.getTranslation(), drive.getFieldSpeeds(), Seconds.of(0.5)/*trackingHelpers.calculateTimeOfFlight(getExitLinearVelocity(), Degrees.of(hood.getHoodAngleDegrees(drive, shooterInstance)), getAbsoluteDistance(drive))*/));
        }
        else
          Logger.recordOutput("RealOutputs/ShooterDistance", shooterPose.getTranslation().getDistance(redHubPose.getTranslation()));
          return shooterPose.getTranslation().getDistance(redHubPose.getTranslation());
      }
      else {
        Logger.recordOutput("RealOutputs/ShooterDistance", shooterPose.getTranslation().getDistance(redHubPose.getTranslation()));
        return shooterPose.getTranslation().getDistance(redHubPose.getTranslation());
      }
  }

  public void shootAtVelocity(double velocityRps) {
    shooterMotor1.setControl(new Follower(33, MotorAlignmentValue.Opposed));

      shooterMotor.setControl(
          m_request.withVelocity(velocityRps)
      );
  }

  public double getCurrentRPS() {
    return shooterMotor.getVelocity().getValueAsDouble();
  }

  public double getCorrectedRPS(Drive drive, Hood hood) {
    ShooterPoint sp = Constants.ShooterConstants.interpolate(getShooterDistance(drive, hood)); 
    return sp.velocity();
  }

  public void shootAtSpeed(double speed){
    shooterMotor.setControl(dutyCycleOutForMove.withOutput(speed));
  }
 public double getExitVelocity() {
    double rps = shooterMotor.getVelocity().getValueAsDouble();
    double WheelRadius = 0.05;
    return Math.abs( rps * 2 * Math.PI * WheelRadius); 
  }

  public LinearVelocity getExitLinearVelocity() {
    return LinearVelocity.ofBaseUnits(getExitVelocity(), MetersPerSecond);
  }

  public void stopShooting(){
    shooterMotor.setControl(dutyCycleOutForMove.withOutput(0));
  }

 public double getSpeed() {
    return shooterMotor.getVelocity().getValueAsDouble();
  }

  public double getSpeedDutyCycle() {
    return shooterMotor.getDutyCycle().getValueAsDouble();
  }


      /**
     * Apply an AimSolution from AimingCalculator.
     */
    public void applySolution(AimingCalculator.AimSolution sol) {
        if (!sol.feasible()) {
            // Spin down slowly instead of cutting power — keeps flywheel warm
            shootAtVelocity(-5.0);
            return;
        }
        // shooterRPS is already negative (your Clockwise_Positive inversion)
        shootAtVelocity(sol.shooterRPS());
    }
 
    /**
     * True when flywheel is within 1.5 RPS of its target.
     * Loosen to 2.0 RPS if you're waiting too long; tighten to 1.0 if you're missing.
     */
    public boolean atSetpoint(AimingCalculator.AimSolution sol) {
        return Math.abs(getCurrentRPS() - sol.shooterRPS()) < 1.5;
    }

public boolean readyToFeed(double targetRPS) {
    // targetRPS = -90, threshold = targetRPS + 5 = -85
    // feed when current is AT or PAST -85 (i.e. <= -85)
    return getCurrentRPS() <= targetRPS + 5.0;
}

  @Override
public void periodic() {
    SmartDashboard.putNumber("Shooterleader/CurrentSpeed(Velocity)", getSpeed());
    SmartDashboard.putNumber("Shooterleader/CurrentSpeed(DutyCycle)", getSpeedDutyCycle());

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> applyTunableConfiguration(),
        Tunable.Shooter.LkP, 
        Tunable.Shooter.LkI, 
        Tunable.Shooter.LkD, 
        Tunable.Shooter.LkS, 
        Tunable.Shooter.LkV,
        Tunable.Shooter.LkA, 
        Tunable.Shooter.LmotionMagicCruiseVelocity,
        Tunable.Shooter.LmotionMagicAcceleration,
        Tunable.Shooter.LmotionMagicJerk,
        Tunable.Shooter.LsupplyCurrentLowerTime,
        Tunable.Shooter.LsupplyCurrentLowerLimit,
        Tunable.Shooter.LsupplyCurrentLimit


    );
}
}
