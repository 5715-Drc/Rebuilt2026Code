// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import static edu.wpi.first.units.Units.*;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class FieldTargets {

    // BLUE
    public static final Translation2d BLUE_HUB =
        new Translation2d(4.622, 4.035);

    public static final Translation2d BLUE_PASS1 =
        new Translation2d(2.5, 6.57);

    public static final Translation2d BLUE_PASS2 =
        new Translation2d(2.5, 1.57);

    // RED
    public static final Translation2d RED_HUB =
        new Translation2d(11.915, 4.035);

    public static final Translation2d RED_PASS1 =
        new Translation2d(14.0, 5.57);

    public static final Translation2d RED_PASS2 =
        new Translation2d(14.0, 2.57);
}

  public static class ShooterConstants {
  
public static final double ksystemlatancy = 0.04;
public static final double kvelocityscalar = 1.7
;


      public record ShooterPoint(
          double distance,
          double hoodPos,
          double velocity
      ) {}
static List<ShooterPoint> shooterMap = List.of(
    new ShooterPoint(1.4224316966965267, 0.0, 46.38),
    new ShooterPoint(1.4664231402383262, 0.0, 46.62),
    new ShooterPoint(1.5445472093555506, 0.0005859375, 47.04),
    new ShooterPoint(1.6758554102906762, 0.0005859375, 47.76),
    new ShooterPoint(1.8117182818267707, 0.07353515625, 48.51),
    new ShooterPoint(1.9631075256975268, 0.07353515625, 49.37),
    new ShooterPoint(2.103886286299005, 0.0740234375, 50.18),
    new ShooterPoint(2.180701852127681, 1.16015625, 50.62),
    new ShooterPoint(2.3540853503334045, 1.16015625, 51.65),
    new ShooterPoint(2.4265119224450706, 1.16015625, 52.08),
    new ShooterPoint(2.52644175993728, 1.26513671875, 52.68),
    new ShooterPoint(2.6074124179966405, 1.26513671875, 53.18),
    new ShooterPoint(2.673858594525754, 1.26513671875, 53.59),
    new ShooterPoint(2.8113532196397713, 1.3837890625, 54.44),
    new ShooterPoint(2.9136587869331, 1.66015625, 55.09),
    new ShooterPoint(3.042747259964407, 2.10009765625, 55.91),
    new ShooterPoint(3.1755378257828446, 2.10009765625, 56.77),
    new ShooterPoint(3.2766403223231952, 2.10009765625, 57.44),
    new ShooterPoint(3.349580488799499, 2.10009765625, 57.92),
    new ShooterPoint(3.4803727284071866, 2.10009765625, 58.80),
    new ShooterPoint(3.582452125746096, 2.10009765625, 59.50),
    new ShooterPoint(3.708290736168703, 2.20009765625, 60.37),
    new ShooterPoint(3.835108165157012, 2.30009765625, 61.26),
    new ShooterPoint(3.9291718777628257, 2.5126953125, 61.92),
    new ShooterPoint(4.036142565164119, 2.6459375, 62.69),
    new ShooterPoint(4.126486857959402, 2.72607421875, 63.34),
    new ShooterPoint(4.261843487205129, 2.7355859375, 64.34),
    new ShooterPoint(4.358955483199413, 2.83658203125, 65.06),
    new ShooterPoint(4.4728903930242865, 2.9469921875, 65.92),
    new ShooterPoint(4.542949546871483, 3.0, 69.0),
    new ShooterPoint(4.656386946038429, 3.05, 70.0),
    new ShooterPoint(4.768567463869052, 3.05, 74.00),
    new ShooterPoint(4.872628439591005, 3.05, 76.00),
    new ShooterPoint(4.935631977933902, 3.1, 77.0),
    new ShooterPoint(5.045867181015592, 3.1, 77.5),
    new ShooterPoint(5.1347163686685615, 3.15, 78.00)
);
      public static ShooterPoint interpolate(double distance) {
        for (int i = 0; i < shooterMap.size() - 1; i++) {
            ShooterPoint a = shooterMap.get(i);
            ShooterPoint b = shooterMap.get(i + 1);

            if (distance >= a.distance() && distance <= b.distance()) {
                double t = (distance - a.distance()) /
                          (b.distance() - a.distance());

                double hood = lerp(a.hoodPos(), b.hoodPos(), t);
                double velocity  = lerp(a.velocity(),     b.velocity(),     t);

                return new ShooterPoint(distance, hood, velocity);
            }
        }

        // Clamp to ends
        if (distance < shooterMap.get(0).distance())
          return shooterMap.get(0);

          return shooterMap.get(shooterMap.size() - 1);
        }

        public static double lerp(double a, double b, double t) {
            return a + (b - a) * t;
        }
      }
      public static class TurretConstants {
    
    /**
     * Record for turret increment mapping
     * joystickInput: Joystick value (0.0 - 1.0)
     * tickIncrement: Ticks to add to setpoint
     */
    public record TurretTimeOfFlightPoint(
        double distanceMeters,
        double timeOfFlightSeconds
    ) {}

    /**
     * Lookup table: Distance (meters) → Time of Flight (seconds)
     * Example: 0.3 input = 3 ticks added
     */
    static List<TurretTimeOfFlightPoint> turretTimeOfFlightMap = List.of(
       new TurretTimeOfFlightPoint(1.14, 0.0),//30
          new TurretTimeOfFlightPoint(1.44, 0.0),//60
          new TurretTimeOfFlightPoint(1.74, 0.0),//90
          new TurretTimeOfFlightPoint(2.04, 0),//120
          new TurretTimeOfFlightPoint(2.34, 0),//150
          new TurretTimeOfFlightPoint(2.64, 0),//180
          new TurretTimeOfFlightPoint(2.94, 0),//210
          new TurretTimeOfFlightPoint(3.24, 0),//240
          new TurretTimeOfFlightPoint(3.84, 0),//270
          new TurretTimeOfFlightPoint(4.14, 0),//300
          new TurretTimeOfFlightPoint(4.44, 2.89111328125),//330
          new TurretTimeOfFlightPoint(4.74, 3.08642578125),//360
          new TurretTimeOfFlightPoint(5.04, 3.23046875),//390
          new TurretTimeOfFlightPoint(5.34, 3.23046875)//420
    );

    /**
     * Interpolate tick increment for given joystick input
     */
    public static double getTimeOfFlight(double distanceMeters) {

    if (distanceMeters <= turretTimeOfFlightMap.get(0).distanceMeters()) {
        return turretTimeOfFlightMap.get(0).timeOfFlightSeconds();
    }

    if (distanceMeters >= turretTimeOfFlightMap.get(turretTimeOfFlightMap.size() - 1).distanceMeters()) {
        return turretTimeOfFlightMap.get(turretTimeOfFlightMap.size() - 1).timeOfFlightSeconds();
    }

    for (int i = 0; i < turretTimeOfFlightMap.size() - 1; i++) {
        TurretTimeOfFlightPoint a = turretTimeOfFlightMap.get(i);
        TurretTimeOfFlightPoint b = turretTimeOfFlightMap.get(i + 1);

        if (distanceMeters >= a.distanceMeters() && distanceMeters <= b.distanceMeters()) {
            double t =
                (distanceMeters - a.distanceMeters()) /
                (b.distanceMeters() - a.distanceMeters());

            return a.timeOfFlightSeconds() +
                   (b.timeOfFlightSeconds() - a.timeOfFlightSeconds()) * t;
        }
    }

    return turretTimeOfFlightMap.get(0).timeOfFlightSeconds();
}

 public static class FieldConstants {
        public static final Distance FIELD_LENGTH = Inches.of(650.12);
        public static final Distance FIELD_WIDTH = Inches.of(316.64);

        public static final Distance ALLIANCE_ZONE = Inches.of(156.06);

        public static final Translation3d HUB_BLUE =
                new Translation3d(Inches.of(181.56), FIELD_WIDTH.div(2), Inches.of(56.4));
        public static final Translation3d HUB_RED =
                new Translation3d(FIELD_LENGTH.minus(Inches.of(181.56)), FIELD_WIDTH.div(2), Inches.of(56.4));
        public static final Distance FUNNEL_RADIUS = Inches.of(24);
        public static final Distance FUNNEL_HEIGHT = Inches.of(72 - 56.4);

        public static final Distance TRENCH_BUMP_X =
                Inches.of(181.56); // x position of the center of the trench and bump
        public static final Distance TRENCH_WIDTH = Inches.of(49.86); // y width of the trench
        public static final Distance TRENCH_BUMP_LENGTH = Inches.of(47); // x length of the trench and bump
        public static final Distance TRENCH_BAR_WIDTH = Inches.of(4); // x width of the trench bar
        public static final Distance TRENCH_BLOCK_WIDTH = Inches.of(12); // y width of block separating bump and trench
        public static final Distance BUMP_WIDTH = Inches.of(73); // y width of bump

        public static final Distance TRENCH_CENTER = TRENCH_WIDTH.div(2);
    }
  }
    }
