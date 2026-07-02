// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class IsHubActive extends SubsystemBase {

//     public IsHubActive() {}

//     public boolean isHubActive() {
//     double time = Timer.getMatchTime(); // Seconds remaining in match
//     String gameData = DriverStation.getGameSpecificMessage();
//     var alliance = DriverStation.getAlliance();

//     // 1. Both active during Transition shift, and End Game
//     if (time >= 130 || time <= 30) {
//         // System.out.println("Hub Active: Transition or End Game - TRUE");
//         return true;
//     }


//     // 2. Determine who was assigned "Inactive First"
//     if (gameData.length() > 0 && alliance.isPresent()) {
//         char inactiveFirst = gameData.charAt(0);
//         char myColor = (alliance.get() == DriverStation.Alliance.Red) ? 'R' : 'B';

        
//         // Shift 1 & 3 (120-95s and 70-45s)
//         if ((time <= 130 && time > 105) || (time <= 80 && time > 55)) {
//             boolean active = myColor != inactiveFirst;
//             // System.out.println("Shift 1 & 3: Active = " + active);
//             return active;
//         }
        
//         // Shift 2 & 4 (95-70s and 45-20s)
//         if ((time <= 105 && time > 80) || (time <= 55 && time > 30)) {
//             boolean active = myColor == inactiveFirst;
           
//             return active;
//         }
//     }
    
   
//     return false; // Default to false if data is missing
// }



// public double getTimeToNextShift() {
//     double time = Timer.getMatchTime();

//     double[] shiftBorders = {130, 105, 80, 55, 30, 0};

//     for (double border : shiftBorders) {
//         if (time > border) {
//             return time - border;
//         }
//     }

//     return 0;
// }
//   @Override
//   public void periodic() {

//     SmartDashboard.putBoolean("IsHubActive", isHubActive());
//     SmartDashboard.putNumber("Time To Next Shift", getTimeToNextShift());

//   }
// } 
    package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IsHubActive extends SubsystemBase {

    public IsHubActive() {}

    public boolean isHubActive() {
        double time = Timer.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        var alliance = DriverStation.getAlliance();

        if (time >= 130 || time <= 30) {
            return true;
        }

        if (gameData.length() > 0 && alliance.isPresent()) {
            char inactiveFirst = gameData.charAt(0);
            char myColor = (alliance.get() == DriverStation.Alliance.Red) ? 'R' : 'B';

            boolean isShift1Or3 = (time <= 130 && time > 105) || (time <= 80 && time > 55);
            boolean isShift2Or4 = (time <= 105 && time > 80) || (time <= 55 && time > 30);

            if (isShift1Or3) {
                return myColor != inactiveFirst;
            }
            if (isShift2Or4) {
                return myColor == inactiveFirst;
            }
        }
        return false;
    }

    public double getTimeToNextShift() {
        double time = Timer.getMatchTime();

        if (time > 130) return time - 130;
        if (time > 105) return time - 105;
        if (time > 80) return time - 80;
        if (time > 55) return time - 55;
        if (time > 30) return time - 30;
        return time;
    }

    @Override
    public void periodic() {
        boolean active = isHubActive();
        double timeToShift = getTimeToNextShift();

        if (timeToShift <= 10.0 && Timer.getMatchTime() > 30) {
            boolean blink = (Timer.getFPGATimestamp() % 0.4) < 0.2;
            SmartDashboard.putBoolean("IsHubActive", blink ? active : !active);
        } else {
            SmartDashboard.putBoolean("IsHubActive", active);
        }

        SmartDashboard.putNumber("Time To Next Shift", Math.max(0, timeToShift));
    }
}