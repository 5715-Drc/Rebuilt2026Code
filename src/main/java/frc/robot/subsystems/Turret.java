// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.MetersPerSecond;
// import static edu.wpi.first.units.Units.Radians;
// import static edu.wpi.first.units.Units.Seconds;

// import org.littletonrobotics.junction.Logger;

// import com.ctre.phoenix6.configs.MotionMagicConfigs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Tunable;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.util.LoggedTunableNumber;

// public class Turret extends SubsystemBase {
//   private static Turret TurretInstance = null;
//   private TalonFX m_turret;
//   private DutyCycleOut dutyCycleOutForMove;
//   private TalonFXConfiguration tConfiguration;
//   private MotionMagicConfigs motionMagicConfiguration;
//   final MotionMagicDutyCycle m_request = new MotionMagicDutyCycle(0);
//   double positionToRotate;

//   private final TrackingHelpers trackingHelpers = new TrackingHelpers();

//   // Turret constants
//   public static final double kTurretGearRatio = (1.0/30.0); // motor : turret
//   public static final Rotation2d kTurretZeroOffset = Rotation2d.fromDegrees(0); // adjust after calibration

//   private Transform2d RobotToTurret = new Transform2d(0.148339, -0.127487, Rotation2d.fromDegrees(0));
//   private Pose2d blueHubPose = new Pose2d(4.626, 4.035, new Rotation2d());
//   private Pose2d redHubPose = new Pose2d(11.9, 4.035, new Rotation2d());
//   private Pose2d blueLeftPassPoint = new Pose2d(2.5, 6.07, new Rotation2d());
//   private Pose2d blueRightPassPoint = new Pose2d(2.5, 2.07, new Rotation2d());
//   private Pose2d redLeftPassPoint = new Pose2d(14, 6.07, new Rotation2d());
//   private Pose2d redRightPassPoint = new Pose2d(14, 2.07, new Rotation2d());


//   double OneRotInTicks = 29.76904296875;
//   double OneDegreeInTicks = 0.0826917860243056;

//   private Turret() {

//     m_turret = new TalonFX(41);

//     tConfiguration = new TalonFXConfiguration();
    
//     applyTunableConfiguration();
//     m_turret.getConfigurator().apply(tConfiguration);

//     dutyCycleOutForMove = new DutyCycleOut(0);

//     m_turret.clearStickyFaults();
//     m_turret.setPosition(0);
//   }
  
//   private void applyTunableConfiguration() {
//         // Current limits
//         tConfiguration.CurrentLimits.SupplyCurrentLimit = Tunable.Turret.supplyCurrentLimit.get();
//         tConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
//         tConfiguration.CurrentLimits.StatorCurrentLimit = Tunable.Turret.statorCurrentLimit.get();
//         tConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        
//         tConfiguration.MotorOutput.PeakForwardDutyCycle = Tunable.Turret.PeakForwardDutyCycle.get();
//         tConfiguration.MotorOutput.PeakReverseDutyCycle = Tunable.Turret.PeakReverseDutyCycle.get();

//         tConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Tunable.Turret.maxPosition.get();
//         tConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
//         tConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Tunable.Turret.minPosition.get();
//         tConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
//         // PID Slot0
//         tConfiguration.Slot0.kP = Tunable.Turret.kP.get();
//         tConfiguration.Slot0.kS = Tunable.Turret.kS.get();
//         tConfiguration.Slot0.kV = Tunable.Turret.kV.get();
//         tConfiguration.Slot0.kI = Tunable.Turret.kI.get();

//         tConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//         tConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
//         // Motion Magic
//         motionMagicConfiguration = tConfiguration.MotionMagic;
//         motionMagicConfiguration.MotionMagicCruiseVelocity = Tunable.Turret.cruiseVelocity.get();
//         motionMagicConfiguration.MotionMagicAcceleration = Tunable.Turret.acceleration.get();
//         motionMagicConfiguration.MotionMagicJerk = Tunable.Turret.jerk.get();
        
//         // Apply configuration to motor
//         m_turret.getConfigurator().apply(tConfiguration);
//     }

//   public MotionMagicDutyCycle getMotionMagicRequest() {
//     return m_request;
//   }

//   public static Turret getTurretInstance() {
//     if (TurretInstance == null) TurretInstance = new Turret();
//     return TurretInstance;
//   }

//   public void moveTurret(double speed){
//     dutyCycleOutForMove = new DutyCycleOut(speed);
//     m_turret.setControl(dutyCycleOutForMove);
//   }

//   public void goToPositionMotionMagic(double Goal) {
//     m_turret.setControl(m_request.withPosition(Goal));
//   }

//   public double getTurretPos() {
//     return m_turret.getPosition().getValueAsDouble();
//   }

//   //This method calculates the position of the turret relative to the field 
//   //It uses the position of the turret relative to the robot relative to the field
//   public Pose2d turretToField(Drive drive){
//     double xlocal = RobotToTurret.getX();
//     double ylocal = RobotToTurret.getY();
//     double xGlobal = xlocal * Math.cos(drive.getRotation().getRadians()) - ylocal * Math.sin(drive.getRotation().getRadians()) + drive.getPose().getX();
//     double yGlobal = ylocal * Math.cos(drive.getRotation().getRadians()) + xlocal * Math.sin(drive.getRotation().getRadians()) + drive.getPose().getY();

//     return new Pose2d(xGlobal, yGlobal, drive.getRotation());
//   }

//   //This method calculates the angle from the turret to the target Pose2d
//   public Rotation2d turretRotationToPose(Pose2d pose, Drive drive){
//     Pose2d turretpose = turretToField(drive);
//     double thetaWorldToTarget = Math.atan2((turretpose.getY() - pose.getY()), (turretpose.getX() - pose.getX()));
//     double thetaTurretToTarget = thetaWorldToTarget + Math.PI// adding pi here is an offset
//      - drive.getRotation().getRadians() //subtracting the robot's rotation
//      - (drive.getZSpeedRad() * 0.02);//adding angular velocity lookahead

//     return Rotation2d.fromRadians(thetaTurretToTarget);
//   }

//   public Rotation2d turretRotationToPose(Pose2d targetPose, Drive drive, Shooter shooter){
    
//     Pose2d turretPose = turretToField(drive);

//     double distance = turretPose.getTranslation()
//             .getDistance(targetPose.getTranslation());

//    double timeOfFlight =
//     Constants.TurretConstants.getTimeOfFlight(distance);

//     ChassisSpeeds speeds = drive.getChassisSpeeds();
 
//     ChassisSpeeds fieldSpeeds =
//     ChassisSpeeds.fromRobotRelativeSpeeds(
//         speeds,
//         drive.getRotation()
//     );
//     double dx = fieldSpeeds.vxMetersPerSecond * timeOfFlight;
//     double dy = fieldSpeeds.vyMetersPerSecond * timeOfFlight;

//     Translation2d predictedTarget =
//             targetPose.getTranslation()
//                     .minus(new Translation2d(dx, dy));

//     Translation2d toTarget =
//             predictedTarget.minus(turretPose.getTranslation());

//     double desiredFieldAngle = Math.atan2(toTarget.getY(), toTarget.getX());

//     double robotHeading = drive.getRotation().getRadians();

//     double turretAngleRad =
//             desiredFieldAngle 
//                     - robotHeading
//                     + (drive.getZSpeedRad() * timeOfFlight); // Use ACTUAL flight time

//     turretAngleRad = Math.atan2(Math.sin(turretAngleRad), Math.cos(turretAngleRad));

//     return Rotation2d.fromRadians(turretAngleRad);
//   }
//   public double angleToHub(Drive drive, Shooter shooter, Hood hood) {

//     double dx = 0;
//     double dy = 0;
//     Pose2d hubPose;

//     if(DriverStation.getAlliance().get() == Alliance.Blue){

//     if(drive.getPose().getX() < 4.7)
//         hubPose = blueHubPose;
//     else if (drive.getPose().getY() > 4)
//         hubPose = blueLeftPassPoint;
//           else
//             hubPose = blueRightPassPoint;

//     }
//     else if(DriverStation.getAlliance().get() == Alliance.Red){
//       if(drive.getPose().getX() > 11.8)
//           hubPose = redHubPose;
//       else if (drive.getPose().getY() > 4)
//           hubPose = redRightPassPoint;
//             else
//               hubPose = redLeftPassPoint;
//     }
//     else
//       return 0;

//     if (turretToField(drive).getY() > 4) {
//       if(drive.getChassisSpeeds().vxMetersPerSecond > 0.1 || drive.getChassisSpeeds().vxMetersPerSecond < -0.1 || drive.getChassisSpeeds().vyMetersPerSecond > 0.1 || drive.getChassisSpeeds().vyMetersPerSecond < -0.1){
//         dx = trackingHelpers.predictTargetPos(hubPose.getTranslation(), drive.getFieldSpeeds(), Seconds.of(1)).getX() - turretToField(drive).getX();
//         dy = trackingHelpers.predictTargetPos(hubPose.getTranslation(), drive.getFieldSpeeds(), Seconds.of(1)).getY() - turretToField(drive).getY();
//       }
//         else{
//         dx = hubPose.getX() - turretToField(drive).getX();
//         dy = hubPose.getY() - turretToField(drive).getY();
//         }
//     }
//     else if (turretToField(drive).getY() < 4) {

//       if(drive.getChassisSpeeds().vxMetersPerSecond > 0.1 || drive.getChassisSpeeds().vxMetersPerSecond < -0.1 || drive.getChassisSpeeds().vyMetersPerSecond > 0.1 || drive.getChassisSpeeds().vyMetersPerSecond < -0.1){
//         dx = trackingHelpers.predictTargetPos(hubPose.getTranslation(), drive.getFieldSpeeds(), Seconds.of(1)).getX() - turretToField(drive).getX();
//         dy = turretToField(drive).getY() - trackingHelpers.predictTargetPos(hubPose.getTranslation(), drive.getFieldSpeeds(), Seconds.of(1)).getY();
//       }
//       else{
//       dx = hubPose.getX() - turretToField(drive).getX();
//       dy = turretToField(drive).getY() - hubPose.getY();
//       }
//     }

//     double angle = Math.toDegrees(Math.atan2(dy, dx));
//     return angle;
//   }
 
//   public double getOverComeNAngleToHubTicks(Drive drive, Shooter shooter, Hood hood) {

//   double desiredRotation = -1 * drive.getRotation().getDegrees();
//   double ticks = 0;      

//   if (turretToField(drive).getY() < 4) {
//     ticks = (desiredRotation - angleToHub(drive, shooter, hood)) * OneDegreeInTicks;
//   } 
//   else if (turretToField(drive).getY() > 4) {
//     ticks = (desiredRotation + angleToHub(drive, shooter, hood)) * OneDegreeInTicks;
//   }

//   return ticks;

//     // double targetTicks = clamp(ticks, -14, 14);

//     // double error = targetTicks - getTurretPos();
//     // return error;
//   }

//   public double clamp(double value, double min, double max){
//     return Math.max(min, Math.min(value, max));
//   }
 
//     public void aimTurretAtPoint(Pose2d pose, Drive drive) {
//       m_turret.setControl(m_request.withPosition(turretRotationsToKraken(turretRotationToPose(pose, drive).getRotations())));
//   }

//   public double aimTurretAtPointMoving( Drive drive, Shooter shooter) {
//     Pose2d hubPose;
//     if(DriverStation.getAlliance().get() == Alliance.Blue)
//       hubPose = blueHubPose;
//     else
//       hubPose = redHubPose;
//     Rotation2d desiredAngle = turretRotationToPose(hubPose, drive, shooter);
//    // double motorRotations = desiredAngle.getRotations() * kTurretGearRatio;
//    double ticks = desiredAngle.getRotations() * OneRotInTicks;
//     return ticks;
//   }

//   public void aimTurretAtDegree(double degrees){
//     m_turret.setControl(m_request.withPosition(Rotation2d.fromDegrees(degrees).getRotations()));
//   }

//     public double turretRotationsToKraken(double rot){
//     return rot * (1.0/30.0);
//   }


//     public Rotation2d getTurretAngle() {
//         double motorRot =
//                 getTurretPos();

//         double turretRot =
//                 motorRot / kTurretGearRatio;

//         double turretRad =
//                 turretRot * 2.0 * Math.PI;

//         return new Rotation2d(turretRad)
//                 .minus(kTurretZeroOffset);
//     }

// public double setPointrot() {
// double currentMotorRot = getTurretPos();

// // desired angle → motor rotations
// double desiredMotorRot =
//         getTurretPos()
//         / (2.0 * Math.PI)
//         * (1.0/30.0);

// // shortest-path correction
// double delta =
//         Math.IEEEremainder(
//             desiredMotorRot - currentMotorRot,
//             (1.0/30.0)
//         );

// double finalSetpoint =
//         currentMotorRot + delta;

//         return finalSetpoint;
// }

// public double getTurretAngleDegrees(Drive drive, Shooter shooter) {

//     Pose2d pose = drive.getPose();
//     Translation2d robotPos = pose.getTranslation();

//     Translation2d hub = Constants.TurretConstants.FieldConstants.HUB_BLUE.toTranslation2d();

//     double distance = robotPos.getDistance(hub);

//     double ballSpeed = shooter.getExitLinearVelocity().in(MetersPerSecond);

//     double flightTime = distance / ballSpeed;

//     ChassisSpeeds speeds = drive.getFieldSpeeds();

//     double dx = speeds.vxMetersPerSecond * flightTime;
//     double dy = speeds.vyMetersPerSecond * flightTime;

//     Translation2d predictedHub =
//             hub.minus(new Translation2d(dx, dy));

//     Translation2d diff = predictedHub.minus(robotPos);

//     Rotation2d turretAngle = new Rotation2d(diff.getX(), diff.getY());

//     return turretAngle.getDegrees() * OneDegreeInTicks;
// }


//   @Override
//   public void periodic() {
//     Logger.recordOutput("Turret Position", m_turret.getPosition().getValueAsDouble());

//         LoggedTunableNumber.ifChanged(
//             hashCode(),
//             () -> applyTunableConfiguration(),
//             Tunable.Turret.kP, 
//             Tunable.Turret.kI, 
//             Tunable.Turret.kS, 
//             Tunable.Turret.kV,
//             Tunable.Turret.cruiseVelocity,
//             Tunable.Turret.acceleration,
//             Tunable.Turret.jerk);
//   }
// }
