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
        // PID Gains
        public static final double kP = 0.08;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.023008;
        public static final double kV = 0.115;
        public static final double kA = 0.0;

        // Current Limits
        public static final double supplyCurrentLimit = 40;
        public static final double supplyCurrentLowerLimit = 40;
        public static final double supplyCurrentLowerTime = 0.2;

        // Motion Magic
        public static final double motionMagicCruiseVelocity = 120;
        public static final double motionMagicAcceleration = 400;
        public static final double motionMagicJerk = 4000;

        // Position offsets
        public static final double robotToShooterX = 0.155;
        public static final double robotToShooterY = -0.145;

        //Shooter Velocity
        public static final double shooterVelocity = -50;
    }

    /**
     * Intake subsystem tunable parameters
     */
   public static class Intake {
    // PID Gains

   
    public static final double kP = 0.44;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.2;
    public static final double kV = 0.5;

    // Current Limits
    public static final double supplyCurrentLimit = 30;
    public static final double statorCurrentLimit = 30;

        public static final double maxPosition = -9.45;
        public static final double minPosition = 0;
    // Motion Magic
    public static final double cruiseVelocity = 30;
    public static final double acceleration = 50;
    public static final double jerk = 4000;


    //Move--------------------------------------------Motor
    //PID Gains
    public static final double MkP = 0.15;
    public static final double MkI = 0.0;
    public static final double MkD = 0.0;
    public static final double MkS = 0.49;
    public static final double MkV = 0.105;
    public static final double MKA = 0.0;

    // Motion Magic
    public static final double McruiseVelocity = 115;
    public static final double Macceleration = 80;
    public static final double Mjerk = 1200;
}

    /**
     * Hood subsystem tunable parameters
     */
    public static class Hood {
          
        // PID Gains
        public static final double kP = 0.532;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.4;
        public static final double kV = 0.02325;

        public static final double Max = 0.0;
        // Motion Magic
        public static final double cruiseVelocity = 40;
        public static final double acceleration = 60;
        public static final double jerk = 1000;
    }

    /**
     * Turret subsystem tunable parameters
     */
    public static class Turret {
        // PID Gains
        public static final double kP = 0.2;
        public static final double kI =  0.2;
        public static final double kD = 1; //0.0
        public static final double kS = 0.016;
        public static final double kV = 0.03; //0.005

        // Motion Magic
        public static final double cruiseVelocity = 100;
        public static final double acceleration = 400;
        public static final double jerk = 2400;

        // Current Limits
        public static final double supplyCurrentLimit = 40;
        public static final double statorCurrentLimit = 40;
        public static final double maxPosition = 14;
        public static final double minPosition = -14;
        public static final double PeakForwardDutyCycle = 0.2;
        public static final double PeakReverseDutyCycle = -0.2;

    }

    /**
     * Feeder subsystem tunable parameters
     */
    public static class Feeder {
        // Current Limits
        public static final double supplyCurrentLimit = 30;
        
        // Speed multiplier for fine-tuning
        public static final double speedMultiplier = 1.0;

                // PID Gains
        public static final double kP = 0.07000000029802322;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.19296;
        public static final double kV = 0.11500000208616257;
        public static final double kA = 0.0;

        // Motion Magic
        public static final double motionMagicCruiseVelocity = 120;
        public static final double motionMagicAcceleration = 400;
        public static final double motionMagicJerk = 4000;

        //Feeder Velocity
        public static final double FeederVelocity = -50;
    }
    public static class Indexer {
        // Current Limits
        public static final double supplyCurrentLimit = 30;

        
                // PID Gains
        public static final double kP = 0.02;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.3;
        public static final double kV = 0.1165;
        public static final double kA = 0.0;
        
         // Motion Magic
            public static final double motionMagicCruiseVelocity = 120;
            public static final double motionMagicAcceleration = 400;
            public static final double motionMagicJerk = 4000;
    
            //Indexer Velocity
            public static final double IndexerVelocity = -50;
        
    }

    /**
     * Drive subsystem tunable parameters
     */
    public static class Drive {
        // Auto-aiming
        public static final double autoAimKp = 0.1;
        public static final double autoAimMaxOmega = 3.0;

                // PID Gains
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        
    }

    
}