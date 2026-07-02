package frc.robot.subsystems;


import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Tunable;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class Hood extends SubsystemBase {
  private static Hood HoodInstance = null;
  private TalonFX HoodMotor;
  private DutyCycleOut dutyCycleOutForMove;
  private TalonFXConfiguration HoodConfiguration;
  private MotionMagicConfigs motionMagicConfiguration;
  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
  double positionToRotate;

  private final double MAX_TICKS = 3.0;
  private final double MIN_TICKS = 0.0;

  private double manualTargetPosition = 0.0;
private boolean isManualMode = false;


  private final double oneDergreeInTicks = 0.1339863933343103;

  private Transform2d RobotToHood = new Transform2d(0.155, -0.145, Rotation2d.fromDegrees(0));

  private Hood() {

    HoodMotor = new TalonFX(37);

    HoodConfiguration = new TalonFXConfiguration();
   

    HoodConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    HoodConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    applyTunableConfiguration();
    HoodMotor.getConfigurator().apply(HoodConfiguration);

    dutyCycleOutForMove = new DutyCycleOut(0);

    HoodMotor.clearStickyFaults();
    HoodMotor.setPosition(0);
  }
  private void applyTunableConfiguration() {
       
        // PID Slot0
        HoodConfiguration.Slot0.kP = Tunable.Hood.kP;
        HoodConfiguration.Slot0.kS = Tunable.Hood.kS;
        HoodConfiguration.Slot0.kV = Tunable.Hood.kV;
        HoodConfiguration.Slot0.kI = Tunable.Hood.kI;

    HoodConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    HoodConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
    HoodConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_TICKS;
    HoodConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_TICKS;
    HoodConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    HoodConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    HoodConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
    HoodConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

    
    HoodConfiguration.MotorOutput.PeakForwardDutyCycle = 0.5;
    HoodConfiguration.MotorOutput.PeakReverseDutyCycle = -0.5;

        // Motion Magic
        motionMagicConfiguration = HoodConfiguration.MotionMagic;
        motionMagicConfiguration.MotionMagicCruiseVelocity = Tunable.Hood.cruiseVelocity;
        motionMagicConfiguration.MotionMagicAcceleration = Tunable.Hood.acceleration;
        motionMagicConfiguration.MotionMagicJerk = Tunable.Hood.jerk;
        
        // Apply configuration to motor
        HoodMotor.getConfigurator().apply(HoodConfiguration);
    }

  public static Hood getHoodInstance() {
    if (HoodInstance == null) HoodInstance = new Hood();
    return HoodInstance;
  }

  public void lockCurrentPosition() {
    // Stop the raw motor speed output
    this.HoodMove(0); 
    
    // Grab where the encoder is sitting exactly right now (e.g., 3.6)
    this.manualTargetPosition = getHoodPosition(); 
    
    // Re-engage position holding mode, locked onto the new position
    this.isManualMode = true; 
}

  



  public void ResetHoodPose(){
        HoodMotor.setPosition(0);

  }

    public void HoodMove(double speed){
    HoodMotor.setControl(dutyCycleOutForMove.withOutput(speed));
  }

  public void goToPositionMotionMagic(double Goal) {
    HoodMotor.setControl(m_request.withPosition(Goal));
  }

  public double DegreesToTicks(double degrees) {
    return degrees * oneDergreeInTicks;
  }

    public double TicksToDegrees(double ticks) {
    return ticks / oneDergreeInTicks;
  }

      //This method calculates the position of the hood relative to the field 
  //It uses the position of the hood relative to the robot relative to the field
  public Pose2d hoodToField(Drive drive){
    double xlocal = RobotToHood.getX();
    double ylocal = RobotToHood.getY();
    double xGlobal = xlocal * Math.cos(drive.getRotation().getRadians()) + ylocal * Math.sin(drive.getRotation().getRadians()) + drive.getPose().getX();
    double yGlobal = ylocal * Math.cos(drive.getRotation().getRadians()) + xlocal * Math.sin(drive.getRotation().getRadians()) + drive.getPose().getY();

    return new Pose2d(xGlobal, yGlobal, drive.getRotation());
  }


  public double getHoodPosition() {
    return HoodMotor.getPosition().getValueAsDouble();
  }

      /**
     * Apply an AimSolution from AimingCalculator.
     */
    public void applySolution(AimingCalculator.AimSolution sol) {
        if (!sol.feasible()) return;
        goToPositionMotionMagic(sol.hoodTicks());
    }
 
    /**
     * True when hood is within tolerance of its target.
     * 0.05 ticks ≈ 0.37 deg — tighten if the flywheel is fast enough to tolerate it.
     */
    public boolean atSetpoint(AimingCalculator.AimSolution sol) {
        return Math.abs(getHoodPosition() - sol.hoodTicks()) < 0.05;
    }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Position", HoodMotor.getPosition().getValueAsDouble());






  }
}