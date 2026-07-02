package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class blinkin extends SubsystemBase {
    public static Spark blinkin;


    public blinkin() {
        blinkin = new Spark(0);
  }

  public static void setSolidGreen() {
    blinkin.set(0.77);
  }

  public static void setHeartBeatBlue() {
    blinkin.set(0.07);
  }

  public static void setHeartBeatWhite() {
    blinkin.set(-0.21);
  }

    public static void setBreathingWhite() {
    blinkin.set(0.29);
  }

  public static void setSolidRed() {
    blinkin.set(0.61);
  }

  @Override
  public void periodic() {

  }
}
