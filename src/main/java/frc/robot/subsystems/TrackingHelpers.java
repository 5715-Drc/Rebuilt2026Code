package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants.FieldConstants;

public class TrackingHelpers extends SubsystemBase {

    private Transform3d RobotToTurret = new Transform3d(0.148339, -0.127487, 0.315, Rotation3d.kZero);
    private final static Distance FLY_WHEEL_RADIUS = Inches.of(2);
    private final static Angle MAX_TURN_ANGLE = Degrees.of(180);
    private final static Angle MIN_TURN_ANGLE = Degrees.of(-180);
    
    
      public TrackingHelpers() {
        
        
      }
    
          public Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
            return Meters.of(robot.getTranslation().getDistance(target.toTranslation2d()));
          }
    
          public Angle calculateAngleFromVelocity(Pose2d robot, LinearVelocity velocity, Translation3d target) {

                double g = MetersPerSecondPerSecond.of(9.81).in(InchesPerSecondPerSecond);
                double vel = velocity.in(InchesPerSecond);

                double x_dist = getDistanceToTarget(robot, target).in(Inches);

                double y_dist = target.getMeasureZ()
                        .minus(RobotToTurret.getMeasureZ())
                        .in(Inches);

                double inside = Math.pow(vel, 4) - g * (g * x_dist * x_dist + 2 * y_dist * vel * vel);

                inside = Math.max(inside, 0);   // prevents NaN

                double angle = Math.atan(((vel * vel) + Math.sqrt(inside)) / (g * x_dist));

                System.out.println("velocity: " + vel);
                System.out.println("distance: " + x_dist);
                System.out.println("inside: " + inside);

                return Radians.of(angle);
            }


          // calculates how long it will take for a projectile to travel a set distance given its initial velocity and angle
          public Time calculateTimeOfFlight(LinearVelocity exitVelocity, Angle hoodAngle, Distance distance) {
              double vel = exitVelocity.in(MetersPerSecond);
              double angle = Math.PI / 2 - hoodAngle.in(Degrees);
              double dist = distance.baseUnitMagnitude();
              return Seconds.of(dist / (vel * Math.cos(angle)));
          }
    
          // Move a target a set time in the future along a velocity defined by fieldSpeeds
          public Translation2d predictTargetPos(Translation2d target, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {
              double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight.in(Seconds);
              double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight.in(Seconds);
    
              return new Translation2d(predictedX, predictedY);
          }

          // Move a target a set time in the future along a velocity defined by fieldSpeeds
        //   public Pose2d predictTargetT(Pose2d target, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {
        //       double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight.in(Seconds);
        //       double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight.in(Seconds);
    
        //       return new Pose2d(predictedX, predictedY, target.getRotation());
        //   }
         public Pose2d predictTargetT(Pose2d target, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {

            double totallookaheadtime = timeOfFlight.in(Seconds) + ShooterConstants.ksystemlatancy;
              double predictedX = target.getX() - (fieldSpeeds.vxMetersPerSecond * totallookaheadtime * ShooterConstants.kvelocityscalar);
              double predictedY = target.getY() - (fieldSpeeds.vyMetersPerSecond * totallookaheadtime * ShooterConstants.kvelocityscalar);
    
              return new Pose2d(predictedX, predictedY, target.getRotation());
          }

        // calculates the angle of a turret relative to the robot to hit a target
        public Angle calculateAzimuthAngle(Pose2d robot, Translation3d target, Angle currentAngle) {
            Translation2d turretTranslation = new Pose3d(robot)
                    .transformBy(RobotToTurret)
                    .toPose2d()
                    .getTranslation();

            Translation2d direction = target.toTranslation2d().minus(turretTranslation);
            return calculateAzimuthAngle(robot, direction.getAngle().getMeasure(), currentAngle);
        }

        // calculates the angle of a turret relative to the robot to hit a target
        public Angle calculateAzimuthAngle(Pose2d robot, Angle fieldRelativeAngle, Angle currentAngle) {
            double angle = MathUtil.inputModulus(
                    new Rotation2d(fieldRelativeAngle).minus(robot.getRotation()).getRotations(), -0.5, 0.5);
            double current = currentAngle.in(Rotations);
            if (current > 0 && angle + 1 <= MAX_TURN_ANGLE.in(Rotations)) angle += 1;
            if (current < 0 && angle - 1 >= MIN_TURN_ANGLE.in(Rotations)) angle -= 1;
            Logger.recordOutput("Turret/DesiredAzimuthRad", angle);
            return Rotations.of(angle);
        }

          public static AngularVelocity linearToAngularVelocity(LinearVelocity vel, Distance radius) {
              return RadiansPerSecond.of(vel.in(MetersPerSecond) / radius.in(Meters) / 0.54);
          }

          public static LinearVelocity angularToLinearVelocity(AngularVelocity vel, Distance radius) {
              return MetersPerSecond.of(vel.in(RadiansPerSecond) * radius.in(Meters) * 0.54);
          }
    
          public record ShotData(double exitVelocity, double hoodAngle, Translation3d target) {
            public ShotData(AngularVelocity exitVelocity, Angle hoodAngle, Translation3d target) {
                this(exitVelocity.in(RadiansPerSecond), hoodAngle.in(Radians), target);
            }
    
            public ShotData(AngularVelocity exitVelocity, Angle hoodAngle) {
                this(exitVelocity, hoodAngle, FieldConstants.HUB_BLUE);
            }
    
            public ShotData(double exitVelocity, double hoodAngle) {
                this(exitVelocity, hoodAngle, FieldConstants.HUB_BLUE);
            }
    
            public LinearVelocity getLinearExitVelocity() {
                return angularToLinearVelocity(RadiansPerSecond.of(this.exitVelocity), FLY_WHEEL_RADIUS);
        }

        public AngularVelocity getAngularExitVelocity() {
            return RadiansPerSecond.of(this.exitVelocity);
        }

    }


  @Override
  public void periodic() { 

  }
}
