package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utils.ShooterConfig;
import frc.robot.utils.ShooterPreset;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants9128 extends Constants {
  public final Mode currentMode = Mode.REAL;

  public enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  public ShooterConfig getLookupTable(){
        ShooterConfig shooterConfig = new ShooterConfig();
        shooterConfig.getShooterConfigs().add(new ShooterPreset(6, 2500, 2500, 0.85)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(9.25, 2500, 3500, 17.27)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(10.75, 2500, 3500, 33.6)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(11.5, 2500, 4500, 40)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(12, 2500, 4500, 50.4)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(13, 2500, 5000, 64)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(14, 2500, 5500, 85.1)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(14.4, 3500, 5500, 95.5)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(15, 3500, 6000, 112.8)); // Distance -> Bumper
        return shooterConfig;
    }

  public final class LimelightConstants {
    public final String photonVisionName = "9752limelight";
    public final double limelightMountAngleDegrees = 22.3;
    public final double limelightLensHeightInches = 19;
    public final double goalHeightInches = 57;
    public final double cameraToSpeakerDistance = 44;
    public final double turnPID = 0.005;
    public final double heightToAprilTag = 38.625; // Dist from Ll to AprilTag
    public final double aprilTagToSpeakerHeight = 20;

    public final class Blue {
      public final double LOWEST_TY =
          -24.85; // TODO: CALCIBRATE THIS LIMELIGHT VALUE <LOWEST_TY,LOWEST_DISTANCE>
      public final double HIGHEST_TY =
          24.85; // TODO: CALIBRATE THIS LIMELIGHT VALUE <HIGHEST_TY,HIGHEST_DISTANCE>

      public final Map<Double, Double> getBlueDistanceMap() {
        Map<Double, Double> distanceMap = new HashMap<>();
        distanceMap.clear();
        distanceMap.put(LOWEST_TY, LOWEST_DISTANCE);
        distanceMap.put(HIGHEST_TY, HIGHEST_DISTANCE);
        distanceMap.put(0.0, 100.0); // TODO: CALIBRATE THESE VALUES
        return distanceMap;
      }

      public final double LOWEST_DISTANCE =
          80.0; // TODO: CALCIBRATE THIS LIMELIGHT VALUE <LOWEST_TY,LOWEST_DISTANCE>
      public final double HIGHEST_DISTANCE =
          120.0; // TODO: CALIBRATE THIS LIMELIGHT VALUE <HIGHEST_TY,HIGHEST_DISTANCE>
    }

    public final class Red {
      public final double LOWEST_TY =
          -24.85; // TODO: CALCIBRATE THIS LIMELIGHT VALUE <LOWEST_TY,LOWEST_DISTANCE>
      public final double HIGHEST_TY =
          24.85; // TODO: CALIBRATE THIS LIMELIGHT VALUE <HIGHEST_TY,HIGHEST_DISTANCE>

      public final Map<Double, Double> getRedDistanceMap() {
        Map<Double, Double> distanceMap = new HashMap<>();
        distanceMap.clear();
        distanceMap.put(LOWEST_TY, LOWEST_DISTANCE);
        distanceMap.put(HIGHEST_TY, HIGHEST_DISTANCE);
        distanceMap.put(0.0, 100.0); // TODO: CALIBRATE THIS LIMELIGHT VALUE <TY,DISTANCE>
        return distanceMap;
      }

      public final double LOWEST_DISTANCE =
          80.0; // TODO: CALCIBRATE THIS LIMELIGHT VALUE <LOWEST_TY,LOWEST_DISTANCE>
      public final double HIGHEST_DISTANCE =
          120.0; // TODO: CALIBRATE THIS LIMELIGHT VALUE <HIGHEST_TY,HIGHEST_DISTANCE>
    }
  }
  /* As per old code, given by Coach Abdurrehman ^^^ */

  public final class SwerveConstants {
    public final double headingPresetKp = 0.003*5.0;
    public final double headingPresetKd = 0.0001*9.0;
    public final double driveHeadingKp = 0.003*5.0;
    public final double driveHeadingKd = 0.0001*9.0;

    public final double translationalAutoP = 3.8;
    public final double rotationalAutoP = 6.25;

    public final int mod0Drive = 1;
    public final int mod0Turn = 2;
    public final int mod0Cancoder = 3;

    public final int mod1Drive = 4;
    public final int mod1Turn = 5;
    public final int mod1Cancoder = 6;

    public final int mod2Drive = 7;
    public final int mod2Turn = 8;
    public final int mod2Cancoder = 9;

    public final int mod3Drive = 10;
    public final int mod3Turn = 11;
    public final int mod3Cancoder = 12;
  }

  public double degreesToArmAngle(double degrees, double gearRatio) {
    return degrees * 56.1 / 90; // Returns rotations
  }

  public final class IntakeConstants {
    public final int kIntakeMotorPort = 15;
  }

  public final class ArmConstants {
    public final int kArmMotorPort = 16;
    public final double ampPos = 70;
    public final double subPos = 10;

    public class ArmPIDConstants {
      // ArmPID came from Rayyan b/c his worked more smoothly than mine
      public final double kG = 0.255;
      public final double kP = 1.5;
      public final double kI = 0;
      public final double kD = 0.2;
    }
  }

  public final class ShooterConstants {
    public final int kTransferMotorPort = 20;
    public final int kLeftShooterMotorPort = 21; // SparkMAX Controller
    public final int kRightShooterMotorPort = 20; // SparkMAX Controller
    public final int kServoPort = 9;

    public class ShooterPIDConstants {
      public final double kFF = 0.002;
      public final double kP = 0.002;
      public final double kI = 0.0;
      public final double kD = 0.001;
    }
  }

  public final class ControllerConstants {
    public final int IncrementAnimButton = XboxController.Button.kRightBumper.value;
    public final int DecrementAnimButton = XboxController.Button.kLeftBumper.value;
    public final int BlockButton = XboxController.Button.kStart.value;
    public final int MaxBrightnessAngle = 90;
    public final int MidBrightnessAngle = 180;
    public final int ZeroBrightnessAngle = 270;
    public final int VbatButton = XboxController.Button.kA.value;
    public final int V5Button = XboxController.Button.kB.value;
    public final int CurrentButton = XboxController.Button.kX.value;
    public final int TemperatureButton = XboxController.Button.kY.value;
    public final boolean kIsTuningMode = true;
  }

  public final class CANdleConstants {
    public final int CANdleID = 1;
  }

}
