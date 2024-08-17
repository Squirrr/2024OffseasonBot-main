package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utils.ShooterConfig;
import frc.robot.utils.ShooterPreset;
import java.util.HashMap;
import java.util.Map;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
  public static final Mode currentMode = null;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  public static ShooterConfig getLookupTable() {
    ShooterConfig shooterConfig = new ShooterConfig();
    return shooterConfig;
  }

  public static final class LimelightConstants {
    public static final String photonVisionName = null;
    public static final double limelightMountAngleDegrees = 22.3;
    public static final double limelightLensHeightInches = 19;
    public static final double goalHeightInches = 57;
    public static final double cameraToSpeakerDistance = 44;
    public static final double turnPID = 0.005;
    public static final double heightToAprilTag = 38.625; // Dist from Ll to AprilTag
    public static final double aprilTagToSpeakerHeight = 20;

    public static final class Blue {
      public static final double LOWEST_TY =
          -24.85; // TODO: CALCIBRATE THIS LIMELIGHT VALUE <LOWEST_TY,LOWEST_DISTANCE>
      public static final double HIGHEST_TY =
          24.85; // TODO: CALIBRATE THIS LIMELIGHT VALUE <HIGHEST_TY,HIGHEST_DISTANCE>

      public static final Map<Double, Double> getBlueDistanceMap() {
        Map<Double, Double> distanceMap = new HashMap<>();
        distanceMap.clear();
        distanceMap.put(LOWEST_TY, LOWEST_DISTANCE);
        distanceMap.put(HIGHEST_TY, HIGHEST_DISTANCE);
        distanceMap.put(0.0, 100.0); // TODO: CALIBRATE THESE VALUES
        return distanceMap;
      }

      public static final double LOWEST_DISTANCE =
          80.0; // TODO: CALCIBRATE THIS LIMELIGHT VALUE <LOWEST_TY,LOWEST_DISTANCE>
      public static final double HIGHEST_DISTANCE =
          120.0; // TODO: CALIBRATE THIS LIMELIGHT VALUE <HIGHEST_TY,HIGHEST_DISTANCE>
    }

    public static final class Red {
      public static final double LOWEST_TY =
          -24.85; // TODO: CALCIBRATE THIS LIMELIGHT VALUE <LOWEST_TY,LOWEST_DISTANCE>
      public static final double HIGHEST_TY =
          24.85; // TODO: CALIBRATE THIS LIMELIGHT VALUE <HIGHEST_TY,HIGHEST_DISTANCE>

      public static final Map<Double, Double> getRedDistanceMap() {
        Map<Double, Double> distanceMap = new HashMap<>();
        distanceMap.clear();
        distanceMap.put(LOWEST_TY, LOWEST_DISTANCE);
        distanceMap.put(HIGHEST_TY, HIGHEST_DISTANCE);
        distanceMap.put(0.0, 100.0); // TODO: CALIBRATE THIS LIMELIGHT VALUE <TY,DISTANCE>
        return distanceMap;
      }

      public static final double LOWEST_DISTANCE =
          80.0; // TODO: CALCIBRATE THIS LIMELIGHT VALUE <LOWEST_TY,LOWEST_DISTANCE>
      public static final double HIGHEST_DISTANCE =
          120.0; // TODO: CALIBRATE THIS LIMELIGHT VALUE <HIGHEST_TY,HIGHEST_DISTANCE>
    }
  }
  /* As per old code, given by Coach Abdurrehman ^^^ */

  public static final class SwerveConstants {
    public static final double headingPresetKp = 0.003 * 3.0;
    public static final double headingPresetKd = 0.0001 * 3.0;
    public static final double driveHeadingKp = 0.003 * 3.0;
    public static final double driveHeadingKd = 0.0001 * 3.0;

    public static final double translationalAutoP = 3.8;
    public static final double rotationalAutoP = 6.25;

    public static final int mod0Drive = 1;
    public static final int mod0Turn = 2;
    public static final int mod0Cancoder = 3;

    public static final int mod1Drive = 4;
    public static final int mod1Turn = 5;
    public static final int mod1Cancoder = 6;

    public static final int mod2Drive = 7;
    public static final int mod2Turn = 8;
    public static final int mod2Cancoder = 9;

    public static final int mod3Drive = 10;
    public static final int mod3Turn = 11;
    public static final int mod3Cancoder = 12;
  }

  public static double degreesToArmAngle(double degrees, double gearRatio) {
    return degrees * 56.1 / 90; // Returns rotations
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorPort = 15;
  }

  public static final class ArmConstants {
    public static final int kArmMotorPort = 16;
    public static final double ampPos = 70;
    public static final double subPos = 10;

    public class ArmPIDConstants {
      // ArmPID came from Rayyan b/c his worked more smoothly than mine
      public static final double kG = 0.255;
      public static final double kP = 1.5;
      public static final double kI = 0;
      public static final double kD = 0.2;
    }
  }

  public static final class ShooterConstants {
    public static final int kTransferMotorPort = 20;
    public static final int kLeftShooterMotorPort = 21; // SparkMAX Controller
    public static final int kRightShooterMotorPort = 20; // SparkMAX Controller
    public static final int kServoPort = 9;

    public class ShooterPIDConstants {
      public static final double kFF = 0.002;
      public static final double kP = 0.002;
      public static final double kI = 0.0;
      public static final double kD = 0.001;
    }
  }

  public static final class ControllerConstants {
    public static final int IncrementAnimButton = XboxController.Button.kRightBumper.value;
    public static final int DecrementAnimButton = XboxController.Button.kLeftBumper.value;
    public static final int BlockButton = XboxController.Button.kStart.value;
    public static final int MaxBrightnessAngle = 90;
    public static final int MidBrightnessAngle = 180;
    public static final int ZeroBrightnessAngle = 270;
    public static final int VbatButton = XboxController.Button.kA.value;
    public static final int V5Button = XboxController.Button.kB.value;
    public static final int CurrentButton = XboxController.Button.kX.value;
    public static final int TemperatureButton = XboxController.Button.kY.value;
    public static final boolean kIsTuningMode = true;
  }

  public static final class CANdleConstants {
    public static final int CANdleID = 1;
  }

}
