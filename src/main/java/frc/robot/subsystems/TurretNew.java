package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldTargets;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Tunable;
import frc.robot.Tunable.Turret;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class TurretNew extends SubsystemBase {
  private static TurretNew TurretNewInstance = null;
  private TalonFX m_turret;
  private DutyCycleOut dutyCycleOutForMove;
  private TalonFXConfiguration tConfiguration;
  private MotionMagicConfigs motionMagicConfiguration;
  final MotionMagicDutyCycle m_request = new MotionMagicDutyCycle(0);
  double positionToRotate;

  private final TrackingHelpers trackingHelpers = new TrackingHelpers();

  // Turret constants
  public static final double kTurretGearRatio = (1.0/30.0); // motor : turret
  public static final Rotation2d kTurretZeroOffset = Rotation2d.fromDegrees(0); // adjust after calibration
private static final double TURRET_ZERO_OFFSET_ROTATIONS = 0.63;
  private static Transform2d RobotToTurret = new Transform2d(0.148339, 0.127487, Rotation2d.fromDegrees(0));

  double OneRotInTicks = 29.76904296875;
  double OneDegreeInTicks = 0.0826917860243056;

  private TurretNew() {

    m_turret = new TalonFX(41);

    tConfiguration = new TalonFXConfiguration();
    
    applyTunableConfiguration();
    m_turret.getConfigurator().apply(tConfiguration);

    dutyCycleOutForMove = new DutyCycleOut(0);

    m_turret.clearStickyFaults();
    m_turret.setPosition(0);
  }
  
 private void applyTunableConfiguration() {
        // Current limits 
        tConfiguration.CurrentLimits.SupplyCurrentLimit = Tunable.Turret.LsupplyCurrentLimit.get();
        tConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        tConfiguration.CurrentLimits.StatorCurrentLimit = Tunable.Turret.LstatorCurrentLimit.get();
        tConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // Peak Duty Cycle
        tConfiguration.MotorOutput.PeakForwardDutyCycle = Tunable.Turret.LpeakForwardDutyCycle.get();
        tConfiguration.MotorOutput.PeakReverseDutyCycle = Tunable.Turret.LpeakReverseDutyCycle.get();

        // Soft Limits
        tConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Tunable.Turret.LmaxPosition.get();
        tConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        tConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Tunable.Turret.LminPosition.get();
        tConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        // PID Slot0
        tConfiguration.Slot0.kP = Tunable.Turret.LkP.get();
        tConfiguration.Slot0.kS = Tunable.Turret.LkS.get();
        tConfiguration.Slot0.kV = Tunable.Turret.LkV.get();
        tConfiguration.Slot0.kI = Tunable.Turret.LkI.get();
        tConfiguration.Slot0.kD = Tunable.Turret.LkD.get(); 
        tConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        tConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Motion Magic
        motionMagicConfiguration = tConfiguration.MotionMagic;
        motionMagicConfiguration.MotionMagicCruiseVelocity = Tunable.Turret.LcruiseVelocity.get();
        motionMagicConfiguration.MotionMagicAcceleration = Tunable.Turret.Lacceleration.get();
        motionMagicConfiguration.MotionMagicJerk = Tunable.Turret.Ljerk.get();
        
        // Apply configuration to motor
        m_turret.getConfigurator().apply(tConfiguration);
    }

    public MotionMagicDutyCycle getMotionMagicRequest() {
      return m_request;
    }

    public static TurretNew getTurretInstance() {
      if (TurretNewInstance == null) TurretNewInstance = new TurretNew();
      return TurretNewInstance;
    }

    public void moveTurret(double speed){
      dutyCycleOutForMove = new DutyCycleOut(speed);
      m_turret.setControl(dutyCycleOutForMove);
    }

    public void goToPositionMotionMagic(double Goal) {
      m_turret.setControl(m_request.withPosition(Goal));
    }

    public double getTurretPos() {
      return m_turret.getPosition().getValueAsDouble();
    }

    //This method calculates the position of the turret relative to the field 
    //It uses the position of the turret relative to the robot relative to the field
    public static Pose2d turretToField(Drive drive){
      double xlocal = RobotToTurret.getX();
      double ylocal = RobotToTurret.getY();
      double xGlobal = xlocal * Math.cos(drive.getRotation().getRadians()) - ylocal * Math.sin(drive.getRotation().getRadians()) + drive.getPose().getX();
      double yGlobal = ylocal * Math.cos(drive.getRotation().getRadians()) + xlocal * Math.sin(drive.getRotation().getRadians()) + drive.getPose().getY();

      return new Pose2d(xGlobal, yGlobal, drive.getRotation());
    }

public static double getTurretAngle(Pose2d robotPose, Translation2d target) {
    double dx = target.getX() - robotPose.getX();
    double dy = target.getY() - robotPose.getY();
    double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
    double robotHeading = robotPose.getRotation().getDegrees();
    return normalizeAngle(fieldAngle - robotHeading);
}
//     public static double getTurretAngle(Drive drive, Pose2d robotPose, Translation2d target) {

//         double dx = target.getX() - drive.getPose().getX();
//         double dy = target.getY() - drive.getPose().getY();

//         double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));

//         double robotHeading = robotPose.getRotation().getDegrees();

//         double turretAngle = fieldAngle - robotHeading;

//         return normalizeAngle(turretAngle);
// }

    public static double normalizeAngle(double angle) {

        while (angle > 180) {
            angle -= 360;
        }

        while (angle < -180) {
            angle += 360;
        }

        return angle;
    }

    public static Translation2d getClosestPassPoint(
        Pose2d robotPose,
        Translation2d pass1,
        Translation2d pass2
    ) {

        double dist1 = robotPose.getTranslation().getDistance(pass1);
        double dist2 = robotPose.getTranslation().getDistance(pass2);

        if (dist1 < dist2) {
            return pass1;
        }

        return pass2;
    }

    public static boolean inCollectionZone(Pose2d pose) {
      if(DriverStation.getAlliance().get() == Alliance.Blue)
          return pose.getX() > 4.7;
      else
      return pose.getX() < 12.220;
    }

    public static boolean inAllianceZone(Pose2d pose) {
      if(DriverStation.getAlliance().get() == Alliance.Blue)
        return pose.getX() < 4.7;
      else
        return pose.getX() > 12.220;
    }

    public static Translation2d getCurrentTarget(Pose2d robotPose) {

        Alliance alliance = DriverStation.getAlliance().get();

        if (alliance == Alliance.Blue) {

            if (inCollectionZone(robotPose)) {

                return getClosestPassPoint(
                    robotPose,
                    FieldTargets.BLUE_PASS1,
                    FieldTargets.BLUE_PASS2
                );

            } else {

                return FieldTargets.BLUE_HUB;

            }

        } else {

            if (inCollectionZone(robotPose)) {

                return getClosestPassPoint(
                    robotPose,
                    FieldTargets.RED_PASS1,
                    FieldTargets.RED_PASS2
                );

            } else {

                return FieldTargets.RED_HUB;

            }

        }
    }

    public static double turretDegreesToMotorRotations(double degrees) {

        return (degrees / 360.0) * 30.0;
    }

    public double turretDegreesFromMotorRotations() {

        return (m_turret.getPosition().getValueAsDouble() / 30.0) * 360.0;
    }

    // public void aimTurret1(Drive drive) {

    //     Pose2d robotPose = drive.getPose();

    //     // get hub or pass target (your existing method)
    //     Translation2d targetTranslation = getCurrentTarget(robotPose);

    //     Pose2d targetPose =
    //         new Pose2d(targetTranslation, new Rotation2d());

    //     Pose2d finalTarget = targetPose;

    //     ChassisSpeeds speeds = drive.getFieldSpeeds();

    //     boolean robotMoving =
    //         Math.abs(speeds.vxMetersPerSecond) > 0.1 ||
    //         Math.abs(speeds.vyMetersPerSecond) > 0.1;

    //     if (robotMoving) {
    //         double  distanceToTarget = robotPose.getTranslation().getDistance(targetTranslation);
    //         double expectedBallVelocityMPS =12.0;
    //         double dynamicToSeconds = distanceToTarget / expectedBallVelocityMPS;
            
    //         Time tof = Seconds.of(dynamicToSeconds);
    //         finalTarget = trackingHelpers.predictTargetT(
    //             targetPose,
    //             speeds,
    //             tof
    //         );
    //     }

       //  double turretAngle = getTurretAngle(drive ,robotPose, finalTarget.getTranslation());

    //     double motorRotations = turretAngle / 360.0 * 30.0;

    //     Logger.recordOutput("RealOutputs/Turret/CurrentTarget", finalTarget);

    //     m_turret.setControl(
    //         m_request.withPosition(motorRotations - 0.63)
    //     );
    // }
public void aimTurret1(Drive drive) {
    Pose2d currentPose = drive.getPose();
    ChassisSpeeds speeds = drive.getFieldSpeeds();


    double predictedRobotX = currentPose.getX() + (speeds.vxMetersPerSecond * ShooterConstants.ksystemlatancy);
    double predictedRobotY = currentPose.getY() + (speeds.vyMetersPerSecond * ShooterConstants.ksystemlatancy);

    Pose2d robotPose = new Pose2d(predictedRobotX, predictedRobotY, currentPose.getRotation());
    Translation2d targetTranslation = getCurrentTarget(robotPose);

    boolean robotMoving = Math.abs(speeds.vxMetersPerSecond) > 0.1 || Math.abs(speeds.vyMetersPerSecond) > 0.1;
    Translation2d finalTargetTranslation = targetTranslation;

    if (robotMoving) {
        double ballVelocity = 12.0;  
        Translation2d predicted = targetTranslation;

        for (int i = 0; i < 3; i++) {
            double dist = robotPose.getTranslation().getDistance(predicted);
            double tof = dist / ballVelocity;
            double totalLookAhead = tof + ShooterConstants.ksystemlatancy;

            double adjustedX = targetTranslation.getX() - (speeds.vxMetersPerSecond * totalLookAhead * ShooterConstants.kvelocityscalar);
            double adjustedY = targetTranslation.getY() - (speeds.vyMetersPerSecond * totalLookAhead * ShooterConstants.kvelocityscalar);

            predicted = new Translation2d(adjustedX, adjustedY);
        }

        finalTargetTranslation = predicted;
    }

    double turretAngle = getTurretAngle(robotPose, finalTargetTranslation);
    double motorRotations = (turretAngle / 360.0) * 30.0; // نسبة الجير 30 لـ 1

    Logger.recordOutput("Turret/FinalTarget", new Pose2d(finalTargetTranslation, new Rotation2d()));
    Logger.recordOutput("Turret/TurretAngleDeg", turretAngle);
    Logger.recordOutput("Turret/MotorRotations", motorRotations);
    Logger.recordOutput("Turret/RobotSpeed", Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    Logger.recordOutput("Turret/DistToTarget", robotPose.getTranslation().getDistance(targetTranslation));
    Logger.recordOutput("Turret/CompensationMeters", targetTranslation.getDistance(finalTargetTranslation));

    m_turret.setControl(m_request.withPosition(motorRotations - TURRET_ZERO_OFFSET_ROTATIONS));
}

    
        /**
     * Apply an AimSolution from AimingCalculator.
     * Replaces aimTurret1().
     */
    public void applySolution(AimingCalculator.AimSolution sol) {
        if (!sol.feasible()) {
            // Outside physics range — hold current position, don't slew to garbage
            return;
        }
        goToPositionMotionMagic(sol.turretMotorRotations());
    }
     
    /**
     * True when turret is within tolerance of its target.
     * Tune the 0.03 rotations (~0.36 deg at turret) tighter if accuracy allows.
     */
    public boolean atSetpoint(AimingCalculator.AimSolution sol) {
        return Math.abs(getTurretPos() - sol.turretMotorRotations()) < 0.03;
    }


  @Override
  public void periodic() {
    Logger.recordOutput("Turret/PositionDeg", turretDegreesFromMotorRotations());
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> applyTunableConfiguration(),
        Tunable.Turret.LkP,
        Tunable.Turret.LkI,
        Tunable.Turret.LkD,
        Tunable.Turret.LkS,
        Tunable.Turret.LkV,
        Tunable.Turret.LcruiseVelocity,
        Tunable.Turret.Lacceleration,
        Tunable.Turret.Ljerk,
        Tunable.Turret.LsupplyCurrentLimit,
        Tunable.Turret.LstatorCurrentLimit,
        Tunable.Turret.LmaxPosition,
        Tunable.Turret.LminPosition,
        Tunable.Turret.LpeakForwardDutyCycle,
        Tunable.Turret.LpeakReverseDutyCycle
    );
  }
}
