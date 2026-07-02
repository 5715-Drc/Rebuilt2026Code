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

public class Indexer extends SubsystemBase {
    private static Indexer IndexerInstance = null;
    private TalonFX m_indexer;
    private TalonFXConfiguration indexerConfiguration;
    private DutyCycleOut dutyCycleOut;
    private final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
    private  MotionMagicConfigs motionMagicConfiguration;
  private double desiredState = 0;


    private Indexer() {
    m_indexer = new TalonFX(30);
    indexerConfiguration = new TalonFXConfiguration();
    indexerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    indexerConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    applyTunableConfiguration();


    m_indexer.getConfigurator().apply(indexerConfiguration);

    dutyCycleOut = new DutyCycleOut(0);
  }

   private void applyTunableConfiguration() {
               

        indexerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        indexerConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        indexerConfiguration.CurrentLimits.SupplyCurrentLimit = Tunable.Indexer.supplyCurrentLimit;
        indexerConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;        
        
        // PID Slot0
        // indexerConfiguration.Slot0.kP = Tunable.Indexer.kP;
        // indexerConfiguration.Slot0.kI = Tunable.Indexer.kI;
        // indexerConfiguration.Slot0.kD = Tunable.Indexer.kD;
        // indexerConfiguration.Slot0.kS = Tunable.Indexer.kS;
        // indexerConfiguration.Slot0.kV = Tunable.Indexer.kV;
        // indexerConfiguration.Slot0.kA = Tunable.Indexer.kA;

        // Motion Magic configuration
      // motionMagicConfiguration = indexerConfiguration.MotionMagic;

      //   motionMagicConfiguration.MotionMagicCruiseVelocity = Tunable.Indexer.motionMagicCruiseVelocity;
      //   motionMagicConfiguration.MotionMagicAcceleration = Tunable.Indexer.motionMagicAcceleration;
      //   motionMagicConfiguration.MotionMagicJerk = Tunable.Indexer.motionMagicJerk;

        // Apply configuration to motor
        // m_indexer.getConfigurator().apply(indexerConfiguration);
    }


  public static Indexer getIndexerInstance() {
    if (IndexerInstance == null) IndexerInstance = new Indexer();
    return IndexerInstance;
  }
public void indexerAtVelocity(double velocityRps) {
    m_indexer.setControl(m_request.withVelocity(velocityRps));

  }

  public void indexerMove(double value) {
    m_indexer.setControl(dutyCycleOut.withOutput(value));
  }

  public void indexerstop(){
    m_indexer.setControl(dutyCycleOut.withOutput(0));
  }

  public void setStopIndexerState() {
     desiredState = 0;
  }
   public void getIndexerVelocity(){
    desiredState = -0.65;
  }
  public double getIndexerState() {
    return desiredState;
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Indexer DutyCycle", m_indexer.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("Indexer Velocity", m_indexer.getVelocity().getValueAsDouble());
           
  }
}
