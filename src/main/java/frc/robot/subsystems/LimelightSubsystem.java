// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants9752;
import frc.robot.utils.LimelightHelpers;
import java.util.HashMap;
import java.util.Map;

public class LimelightSubsystem extends SubsystemBase {
  private Timer targetSeenTimer = new Timer();
  public double detectedTargetDistance = 0;
  public double limelightArmPos;

  private Map<Double, Double> distanceMap = new HashMap<>();

  public LimelightSubsystem() {
    initializeDistanceMap();
  }

  private void initializeDistanceMap() {
    if (isRedMap()) {
      distanceMap = Constants9752.LimelightConstants.Red.getRedDistanceMap();
    } else { // its a blue map by default
      distanceMap = Constants9752.LimelightConstants.Blue.getBlueDistanceMap();
    }
  }

  public boolean isRedMap() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  public boolean isBlueMap() {
    return !isRedMap();
  }

  @Override
  public void periodic() {
    double ty =
        Double
            .NaN; // at the beginning of periodic we are NaN so we can output if ty is null properly
    if (LimelightHelpers.getTV("limelight")) {
      targetSeenTimer.restart();
      ty = LimelightHelpers.getTY("limelight");
      detectedTargetDistance =
          getDistanceFromTy(ty) - Constants9752.LimelightConstants.cameraToSpeakerDistance;
    } else if (targetSeenTimer.get() > 0.1) {
      detectedTargetDistance = -1;
    }
    DogLog.log("Limelight/TY", ty);
    DogLog.log("Limelight/Detected_Target_Distance", detectedTargetDistance);
    DogLog.log("Limelight/Limelight_Arm_Pos", limelightArmPos);
  }

  private double getDistanceFromTy(double ty) {
    if (distanceMap.isEmpty()) {
      return -1;
    }
    if (ty < Constants9752.LimelightConstants.Red.LOWEST_TY
        || ty < Constants9752.LimelightConstants.Blue.LOWEST_TY) {
      if (isRedMap()) {
        return Constants9752.LimelightConstants.Red.LOWEST_DISTANCE;
      } else {
        return Constants9752.LimelightConstants.Blue.LOWEST_DISTANCE;
      }

    } else if (ty > Constants9752.LimelightConstants.Red.HIGHEST_TY
        || ty > Constants9752.LimelightConstants.Blue.HIGHEST_TY) {
      if (isRedMap()) {
        return Constants9752.LimelightConstants.Red.HIGHEST_DISTANCE;
      } else {
        return Constants9752.LimelightConstants.Blue.HIGHEST_DISTANCE;
      }
    }

    double lowerTy = Double.MAX_VALUE;
    double upperTy = Double.MIN_VALUE;
    double lowerDistance = 0;
    double upperDistance = 0;

    for (Map.Entry<Double, Double> entry : distanceMap.entrySet()) {
      double currentTy = entry.getKey();
      if (currentTy <= ty && currentTy > lowerTy) {
        lowerTy = currentTy;
        lowerDistance = entry.getValue();
      }
      if (currentTy >= ty && currentTy < upperTy) {
        upperTy = currentTy;
        upperDistance = entry.getValue();
      }
    }

    if (lowerTy == Double.MAX_VALUE) {
      return upperDistance;
    }
    if (upperTy == Double.MIN_VALUE) {
      return lowerDistance;
    }

    double slope = (upperDistance - lowerDistance) / (upperTy - lowerTy);

    return lowerDistance + slope * (ty - lowerTy);
  }

  public double setLimelightArmPos() {
    double distance =
        (Constants9752.LimelightConstants.heightToAprilTag
                / Math.tan(Math.toRadians(LimelightHelpers.getTY("limelight")) + 10))
            + 10;

    double lAngle =
        Math.toDegrees(
            Math.atan(
                (Constants9752.LimelightConstants.heightToAprilTag
                        + Constants9752.LimelightConstants.aprilTagToSpeakerHeight)
                    / distance));

    double armAngle = 70 - lAngle;
    double posRatio = 56.1 / 90;
    double limelightPos = armAngle * posRatio;
    return limelightPos;
  }

  public double limelight_aim_proportional() {
    double kP = .0065;

    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
    targetingAngularVelocity *= -1.0;

    if (Math.round(targetingAngularVelocity * 1000) == 0) {
      return 0;
    } else {
      return targetingAngularVelocity;
    }
  }

  public Command DefaultCommand() {
    return run(() -> {});
  }
}
