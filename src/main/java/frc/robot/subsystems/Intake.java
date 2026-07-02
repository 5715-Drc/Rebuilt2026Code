package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Tunable;
import frc.robot.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {
    private static Intake IntakeInstance = null;
    private TalonFX m_move;
    private TalonFX m_extender;
    private DutyCycleOut dutyCycleOut;
    private TalonFXConfiguration config;
    private MotionMagicConfigs magicConfigs;
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    private TalonFXConfiguration MoveConfig;

    final MotionMagicVelocityVoltage m_requestVelocityVoltage = new MotionMagicVelocityVoltage(0);
    private MotionMagicConfigs motionMagicConfigurationMove;


    //constructor
    private Intake(){
        m_move = new TalonFX(31);
        m_extender = new TalonFX(32);
        dutyCycleOut = new DutyCycleOut(0);
        config = new TalonFXConfiguration();
        MoveConfig = new TalonFXConfiguration();



        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        MoveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        MoveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        // Apply initial tunable configuration
        applyTunableConfiguration();
        
        
        //configuration
        m_move.getConfigurator().apply(MoveConfig);
        m_extender.getConfigurator().apply(config);
        m_extender.setPosition(0);
        m_move.clearStickyFaults();
        m_extender.clearStickyFaults();
    }

    /**
     * Apply tunable configuration to motors
     * This method is called on initialization and whenever tunable values change
     */
    private void applyTunableConfiguration() {
    //Extender
        // Current limits
        config.CurrentLimits.SupplyCurrentLimit = Tunable.Intake.LsupplyCurrentLimit.get();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = Tunable.Intake.LstatorCurrentLimit.get();
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // PID Slot0
        config.Slot0.kP = Tunable.Intake.LkP.get();
        config.Slot0.kS = Tunable.Intake.LkS.get();
        config.Slot0.kV = Tunable.Intake.LkV.get();
        config.Slot0.kI = Tunable.Intake.LkI.get();

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Tunable.Intake.LminPosition.get();
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Tunable.Intake.LmaxPosition.get();
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Motion Magic
        magicConfigs = config.MotionMagic;
        magicConfigs.MotionMagicCruiseVelocity = Tunable.Intake.LcruiseVelocity.get();
        magicConfigs.MotionMagicAcceleration = Tunable.Intake.Lacceleration.get();
        magicConfigs.MotionMagicJerk = Tunable.Intake.Ljerk.get();

    //Move
        // PID Slot0
        MoveConfig.Slot0.kP = Tunable.Intake.LMkP.get();
        MoveConfig.Slot0.kS = Tunable.Intake.LMkS.get();
        MoveConfig.Slot0.kV = Tunable.Intake.LMkV.get();
        MoveConfig.Slot0.kI = Tunable.Intake.LMkI.get();
        MoveConfig.Slot0.kD = Tunable.Intake.LMkD.get();
        MoveConfig.Slot0.kA = Tunable.Intake.LMkA.get();

        // Motor configuration
        MoveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        MoveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Motion Magic
        motionMagicConfigurationMove = MoveConfig.MotionMagic;
        motionMagicConfigurationMove.MotionMagicCruiseVelocity = Tunable.Intake.LMcruiseVelocity.get();
        motionMagicConfigurationMove.MotionMagicAcceleration = Tunable.Intake.LMacceleration.get();
        motionMagicConfigurationMove.MotionMagicJerk = Tunable.Intake.LMjerk.get();
        
        // Apply configuration to motors
        m_move.getConfigurator().apply(MoveConfig);
        m_extender.getConfigurator().apply(config);
    }

    public static Intake getIntakeInstance(){
       if (IntakeInstance == null) IntakeInstance = new Intake();
       return IntakeInstance;
    }

    public void ResetIntakePose(){
        m_extender.setPosition(0);
    }
    public void IntakePose(double Goal) {
        m_extender.setControl(m_request.withPosition(Goal));
    }

    public void intakeMove(double speed) {
        m_move.setControl(dutyCycleOut.withOutput(speed));
    }
    public void intakeRotation(double angle) {
        m_extender.setControl(dutyCycleOut.withOutput(angle));
    }

    public void intakeAtVelocity(double velocityRps) {
      m_move.setControl(
          m_requestVelocityVoltage.withVelocity(velocityRps)
      );
    }
    public double getIntakePose() {
        return m_extender.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Position", m_extender.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake/IntakeSpeed", m_move.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake/IntakeTemp",m_move.getDeviceTemp().getValueAsDouble());
        
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> applyTunableConfiguration(),
            // Extender Tunables
            Tunable.Intake.LkP,
            Tunable.Intake.LkI,
            Tunable.Intake.LkS,
            Tunable.Intake.LkV,
            Tunable.Intake.LmaxPosition,
            Tunable.Intake.LminPosition,
            Tunable.Intake.LcruiseVelocity,
            Tunable.Intake.Lacceleration,
            Tunable.Intake.Ljerk,
            // Move Tunables
            Tunable.Intake.LMkP,
            Tunable.Intake.LMkI,
            Tunable.Intake.LMkD,
            Tunable.Intake.LMkS,
            Tunable.Intake.LMkV,
            Tunable.Intake.LMkA,
            Tunable.Intake.LMcruiseVelocity,
            Tunable.Intake.LMacceleration,
            Tunable.Intake.LMjerk,
            // Limits
            Tunable.Intake.LsupplyCurrentLimit,
            Tunable.Intake.LstatorCurrentLimit
        );
    
    }
}