package frc.robot;

import frc.robot.util.LoggedTunableNumber;

/**
 * Central location for all tunable parameters organized by subsystem.
 * Each subsystem has its own nested class containing only essential tunable values.
 */
public class Tunable {

    /**
     * Shooter subsystem tunable parameters
     */
    public static class Shooter {
        public static final LoggedTunableNumber LkP = new LoggedTunableNumber("Shooter/kP", 0.08);
        public static final LoggedTunableNumber LkI = new LoggedTunableNumber("Shooter/kI", 0.0);
        public static final LoggedTunableNumber LkD = new LoggedTunableNumber("Shooter/kD", 0.0);
        public static final LoggedTunableNumber LkS = new LoggedTunableNumber("Shooter/kS", 0.023008);
        public static final LoggedTunableNumber LkV = new LoggedTunableNumber("Shooter/kV", 0.115);
        public static final LoggedTunableNumber LkA = new LoggedTunableNumber("Shooter/kA", 0.0);

        // Motion Magic
        public static final LoggedTunableNumber LmotionMagicCruiseVelocity = new LoggedTunableNumber("Shooter/MotionMagicCruiseVelocity", 120);
        public static final LoggedTunableNumber LmotionMagicAcceleration = new LoggedTunableNumber("Shooter/MotionMagicAccel", 400);
        public static final LoggedTunableNumber LmotionMagicJerk = new LoggedTunableNumber("Shooter/MotionMagicJerk", 4000);

        // Current Limits
        public static final LoggedTunableNumber LsupplyCurrentLimit = new LoggedTunableNumber("Shooter/SupplyCurrentLimit", 40.0); 
        public static final LoggedTunableNumber LsupplyCurrentLowerLimit = new LoggedTunableNumber("Shooter/SupplyCurrentLowerLimit", 35.0);
        public static final LoggedTunableNumber LsupplyCurrentLowerTime = new LoggedTunableNumber("Shooter/SupplyCurrentLowerTime", 0.1); 
    }

    /**
     * Intake subsystem tunable parameters
     */
    public static class Intake {
        // Pivot Motor PID Gains
        public static final LoggedTunableNumber LkP = new LoggedTunableNumber("Intake/kP", 0.44);
        public static final LoggedTunableNumber LkI = new LoggedTunableNumber("Intake/kI", 0.0);
        public static final LoggedTunableNumber LkD = new LoggedTunableNumber("Intake/kD", 0.0);
        public static final LoggedTunableNumber LkS = new LoggedTunableNumber("Intake/kS", 0.2);
        public static final LoggedTunableNumber LkV = new LoggedTunableNumber("Intake/kV", 0.5);

        // Pivot Limits & Motion Magic
        public static final LoggedTunableNumber LmaxPosition = new LoggedTunableNumber("Intake/MaxPosition", -9.45);
        public static final LoggedTunableNumber LminPosition = new LoggedTunableNumber("Intake/MinPosition", 0.0);
        public static final LoggedTunableNumber LcruiseVelocity = new LoggedTunableNumber("Intake/CruiseVelocity", 30.0);
        public static final LoggedTunableNumber Lacceleration = new LoggedTunableNumber("Intake/Acceleration", 50.0);
        public static final LoggedTunableNumber Ljerk = new LoggedTunableNumber("Intake/Jerk", 4000.0);

        // Move Motor PID Gains
        public static final LoggedTunableNumber LMkP = new LoggedTunableNumber("Intake/Move_kP", 0.15);
        public static final LoggedTunableNumber LMkI = new LoggedTunableNumber("Intake/Move_kI", 0.0);
        public static final LoggedTunableNumber LMkD = new LoggedTunableNumber("Intake/Move_kD", 0.0);
        public static final LoggedTunableNumber LMkS = new LoggedTunableNumber("Intake/Move_kS", 0.49);
        public static final LoggedTunableNumber LMkV = new LoggedTunableNumber("Intake/Move_kV", 0.105);
        public static final LoggedTunableNumber LMkA = new LoggedTunableNumber("Intake/Move_kA", 0.0);

        // Move Motor Motion Magic
        public static final LoggedTunableNumber LMcruiseVelocity = new LoggedTunableNumber("Intake/Move_CruiseVelocity", 115.0);
        public static final LoggedTunableNumber LMacceleration = new LoggedTunableNumber("Intake/Move_Acceleration", 80.0);
        public static final LoggedTunableNumber LMjerk = new LoggedTunableNumber("Intake/Move_Jerk", 1200.0);

        // Current Limits
        public static final LoggedTunableNumber LsupplyCurrentLimit = new LoggedTunableNumber("Intake/SupplyCurrentLimit", 30.0);
        public static final LoggedTunableNumber LstatorCurrentLimit = new LoggedTunableNumber("Intake/StatorCurrentLimit", 30.0);
    }

    /**
     * Hood subsystem tunable parameters
     */
    public static class Hood {
        // PID Gains
        public static final LoggedTunableNumber LkP = new LoggedTunableNumber("Hood/kP", 0.532);
        public static final LoggedTunableNumber LkI = new LoggedTunableNumber("Hood/kI", 0.0);
        public static final LoggedTunableNumber LkD = new LoggedTunableNumber("Hood/kD", 0.0);
        public static final LoggedTunableNumber LkS = new LoggedTunableNumber("Hood/kS", 0.4);
        public static final LoggedTunableNumber LkV = new LoggedTunableNumber("Hood/kV", 0.02325);

        public static final LoggedTunableNumber Lmax = new LoggedTunableNumber("Hood/MaxPosition", 0.0);
        
        // Motion Magic
        public static final LoggedTunableNumber LcruiseVelocity = new LoggedTunableNumber("Hood/CruiseVelocity", 40.0);
        public static final LoggedTunableNumber Lacceleration = new LoggedTunableNumber("Hood/Acceleration", 60.0);
        public static final LoggedTunableNumber Ljerk = new LoggedTunableNumber("Hood/Jerk", 1000.0);
    }

    /**
     * Turret subsystem tunable parameters
     */
    public static class Turret {
        // PID Gains
        public static final LoggedTunableNumber LkP = new LoggedTunableNumber("Turret/kP", 0.2);
        public static final LoggedTunableNumber LkI = new LoggedTunableNumber("Turret/kI", 0.2);
        public static final LoggedTunableNumber LkD = new LoggedTunableNumber("Turret/kD", 1.0);
        public static final LoggedTunableNumber LkS = new LoggedTunableNumber("Turret/kS", 0.016);
        public static final LoggedTunableNumber LkV = new LoggedTunableNumber("Turret/kV", 0.03);

        // Motion Magic
        public static final LoggedTunableNumber LcruiseVelocity = new LoggedTunableNumber("Turret/CruiseVelocity", 100.0);
        public static final LoggedTunableNumber Lacceleration = new LoggedTunableNumber("Turret/Acceleration", 400.0);
        public static final LoggedTunableNumber Ljerk = new LoggedTunableNumber("Turret/Jerk", 2400.0);

        // Current Limits & Constraints
        public static final LoggedTunableNumber LsupplyCurrentLimit = new LoggedTunableNumber("Turret/SupplyCurrentLimit", 40.0);
        public static final LoggedTunableNumber LstatorCurrentLimit = new LoggedTunableNumber("Turret/StatorCurrentLimit", 40.0);
        public static final LoggedTunableNumber LmaxPosition = new LoggedTunableNumber("Turret/MaxPosition", 14.0);
        public static final LoggedTunableNumber LminPosition = new LoggedTunableNumber("Turret/MinPosition", -14.0);
        public static final LoggedTunableNumber LpeakForwardDutyCycle = new LoggedTunableNumber("Turret/PeakForwardDutyCycle", 0.2);
        public static final LoggedTunableNumber LpeakReverseDutyCycle = new LoggedTunableNumber("Turret/PeakReverseDutyCycle", -0.2);
    }

    /**
     * Feeder subsystem tunable parameters
     */
    public static class Feeder {
        // PID Gains
        public static final LoggedTunableNumber LkP = new LoggedTunableNumber("Feeder/kP", 0.07000000029802322);
        public static final LoggedTunableNumber LkI = new LoggedTunableNumber("Feeder/kI", 0.0);
        public static final LoggedTunableNumber LkD = new LoggedTunableNumber("Feeder/kD", 0.0);
        public static final LoggedTunableNumber LkS = new LoggedTunableNumber("Feeder/kS", 0.19296);
        public static final LoggedTunableNumber LkV = new LoggedTunableNumber("Feeder/kV", 0.11500000208616257);
        public static final LoggedTunableNumber LkA = new LoggedTunableNumber("Feeder/kA", 0.0);

        // Motion Magic
        public static final LoggedTunableNumber LmotionMagicCruiseVelocity = new LoggedTunableNumber("Feeder/MotionMagicCruiseVelocity", 120.0);
        public static final LoggedTunableNumber LmotionMagicAcceleration = new LoggedTunableNumber("Feeder/MotionMagicAccel", 400.0);
        public static final LoggedTunableNumber LmotionMagicJerk = new LoggedTunableNumber("Feeder/MotionMagicJerk", 4000.0);

        // Constraints & Velocity
        public static final LoggedTunableNumber LsupplyCurrentLimit = new LoggedTunableNumber("Feeder/SupplyCurrentLimit", 30.0);
        public static final LoggedTunableNumber LspeedMultiplier = new LoggedTunableNumber("Feeder/SpeedMultiplier", 1.0);
        public static final LoggedTunableNumber LfeederVelocity = new LoggedTunableNumber("Feeder/FeederVelocity", -50.0);
    }

    /**
     * Indexer subsystem tunable parameters
     */
    public static class Indexer {
        // PID Gains
        public static final LoggedTunableNumber LkP = new LoggedTunableNumber("Indexer/kP", 0.02);
        public static final LoggedTunableNumber LkI = new LoggedTunableNumber("Indexer/kI", 0.0);
        public static final LoggedTunableNumber LkD = new LoggedTunableNumber("Indexer/kD", 0.0);
        public static final LoggedTunableNumber LkS = new LoggedTunableNumber("Indexer/kS", 0.3);
        public static final LoggedTunableNumber LkV = new LoggedTunableNumber("Indexer/kV", 0.1165);
        public static final LoggedTunableNumber LkA = new LoggedTunableNumber("Indexer/kA", 0.0);
        
        // Motion Magic
        public static final LoggedTunableNumber LmotionMagicCruiseVelocity = new LoggedTunableNumber("Indexer/MotionMagicCruiseVelocity", 120.0);
        public static final LoggedTunableNumber LmotionMagicAcceleration = new LoggedTunableNumber("Indexer/MotionMagicAccel", 400.0);
        public static final LoggedTunableNumber LmotionMagicJerk = new LoggedTunableNumber("Indexer/MotionMagicJerk", 4000.0);

        // Constraints & Velocity
        public static final LoggedTunableNumber LsupplyCurrentLimit = new LoggedTunableNumber("Indexer/SupplyCurrentLimit", 30.0);
        public static final LoggedTunableNumber LindexerVelocity = new LoggedTunableNumber("Indexer/IndexerVelocity", -50.0);
    }

    /**
     * Drive subsystem tunable parameters
     */
    public static class Drive {
        // Auto-aiming
        public static final LoggedTunableNumber LautoAimKp = new LoggedTunableNumber("Drive/AutoAimKp", 0.1);
        public static final LoggedTunableNumber LautoAimMaxOmega = new LoggedTunableNumber("Drive/AutoAimMaxOmega", 3.0);

        // PID Gains
        public static final LoggedTunableNumber LkP = new LoggedTunableNumber("Drive/kP", 0.0);
        public static final LoggedTunableNumber LkI = new LoggedTunableNumber("Drive/kI", 0.0);
        public static final LoggedTunableNumber LkD = new LoggedTunableNumber("Drive/kD", 0.0);
        public static final LoggedTunableNumber LkS = new LoggedTunableNumber("Drive/kS", 0.0);
        public static final LoggedTunableNumber LkV = new LoggedTunableNumber("Drive/kV", 0.0);
        public static final LoggedTunableNumber LkA = new LoggedTunableNumber("Drive/kA", 0.0);
    }
}