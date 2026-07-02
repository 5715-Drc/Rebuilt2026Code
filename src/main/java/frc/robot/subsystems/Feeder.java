package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Tunable;

public class Feeder extends SubsystemBase { 
  private static Feeder feederInstance = null;

  private TalonFX m_feeder;
  private TalonFXConfiguration feederConfiguration;
  private DutyCycleOut dutyCycleOut;

  final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
  private MotionMagicConfigs motionMagicConfiguration;
  private double desiredState = 0;
  private Feeder() {
    m_feeder = new TalonFX(36);
    feederConfiguration = new TalonFXConfiguration();

    feederConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    feederConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;


    applyTunableConfiguration();
    m_feeder.getConfigurator().apply(feederConfiguration);


    dutyCycleOut = new DutyCycleOut(0);
  }
  private void applyTunableConfiguration() {
               

        feederConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        feederConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        feederConfiguration.CurrentLimits.SupplyCurrentLimit = Tunable.Feeder.supplyCurrentLimit;
        feederConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;        
        
        // PID Slot0
        // feederConfiguration.Slot0.kP = Tunable.Feeder.kP;
        // feederConfiguration.Slot0.kS = Tunable.Feeder.kS;
        // feederConfiguration.Slot0.kV = Tunable.Feeder.kV;
        // feederConfiguration.Slot0.kI = Tunable.Feeder.kI;
        // feederConfiguration.Slot0.kD = Tunable.Feeder.kD;
        // feederConfiguration.Slot0.kA = Tunable.Feeder.kA;

        // Motor configuration
        feederConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        feederConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Motion Magic
        // motionMagicConfiguration = feederConfiguration.MotionMagic;
        // motionMagicConfiguration.MotionMagicCruiseVelocity = Tunable.Feeder.motionMagicCruiseVelocity;
        // motionMagicConfiguration.MotionMagicAcceleration = Tunable.Feeder.motionMagicAcceleration;
        // motionMagicConfiguration.MotionMagicJerk = Tunable.Feeder.motionMagicJerk;
        
        // Apply configuration to motor
        // m_feeder.getConfigurator().apply(feederConfiguration);
    }
    public static Feeder getfeederInstance() {
    if (feederInstance == null) feederInstance = new Feeder();
    return feederInstance;
  }

  public void feederMove(double value) {
    m_feeder.setControl(dutyCycleOut.withOutput(value));
  }
  public void feederstop(){
    m_feeder.setControl(dutyCycleOut.withOutput(0));
  }

  public void setStopFeederState() {
    desiredState = 0;
  }
   public void getFeederVelocity(){
    desiredState =  1;
  }
  public double getFeederState() {
    return desiredState;
  }
    public void feedAtVelocity(double velocityRps) {

      m_feeder.setControl(
          m_request.withVelocity(velocityRps)
      );
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Feeder DutyCycle", m_feeder.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("Feeder Velocity", m_feeder.getVelocity().getValueAsDouble());
    
  }
}
