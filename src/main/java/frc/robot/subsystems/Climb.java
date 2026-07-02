// package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.MotionMagicConfigs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Climb extends SubsystemBase {
//   private static Climb ClimbInstance = null;

//   private TalonFX climb_motor;
  

//   private TalonFXConfiguration configuration;
//   private MotionMagicConfigs motionMagicConfiguration;
//   final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

//   private DutyCycleOut dutyCycleOut;

//   private double positionToRotate;

//   private Climb() {
//     climb_motor = new TalonFX(40);

//     configuration = new TalonFXConfiguration();
//     configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//     configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

//     configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 64;
//     configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
//     configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
//     configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

//     configuration.Slot0.kG = 0.3;
//     configuration.Slot0.kV = 0.12; // 0.12 // A velocity target of 1 rps results in 0.12 V output
//     configuration.Slot0.kA = 0.005; // 0.005 // An acceleration of 1 rps/s requires 0.01 V output
//     configuration.Slot0.kP =
//         7.35; // 5.2 // A position error of 2.5 rotations results in 12 V output
//     configuration.Slot0.kI = 0.7; // 0.01 // no output for integrated error
//     configuration.Slot0.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

//     motionMagicConfiguration = configuration.MotionMagic;
//     motionMagicConfiguration.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
//     motionMagicConfiguration.MotionMagicAcceleration =
//         320; // Target acceleration of 160 rps/s (0.5 seconds)
//     motionMagicConfiguration.MotionMagicJerk = 1200; // Target jerk of 1600 rps/s/s (0.1 seconds)

//     climb_motor.getConfigurator().apply(configuration);

//     dutyCycleOut = new DutyCycleOut(0);
    
//     climb_motor.clearStickyFaults();

//     climb_motor.setPosition(0);
//   }

//   public static Climb getClimbInstance() {
//     if (ClimbInstance == null) ClimbInstance = new Climb();
//     return ClimbInstance;
//   }

//   public void goToPositionMotionMagic(double Goal) {
//     positionToRotate = Goal;
//     climb_motor.setControl(m_request.withPosition(Goal));
//   }

//   public void resetPosition() {
//     climb_motor.setPosition(0.0);
//   }

//   public void ClimbMove(double speed) {
//     climb_motor.setControl(dutyCycleOut.withOutput(speed));
//   }

//   public void ClimbStop() {
//     climb_motor.setControl(dutyCycleOut.withOutput(0));
//   }

//   public boolean IsReachedTarget() {
//     return climb_motor.getPosition().getValueAsDouble() > positionToRotate - 1
//         && climb_motor.getPosition().getValueAsDouble() < positionToRotate + 1;
//   }

//   @Override
//   public void periodic() {
//     SmartDashboard.putNumber(
//         "Climb position", climb_motor.getPosition().getValueAsDouble());
//     SmartDashboard.putNumber(
//         "Climb Voltage", climb_motor.getMotorVoltage().getValueAsDouble());
//   }
// }
