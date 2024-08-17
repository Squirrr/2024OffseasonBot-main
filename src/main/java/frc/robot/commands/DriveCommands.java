package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants9752;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static PIDController m_thetaController;

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier xOmegaSupplier,
      DoubleSupplier yOmegaSupplier) {
    m_thetaController = new PIDController(0.00075, 0, 0.00075);
    m_thetaController.enableContinuousInput(-180, 180);

    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);

          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          double xOmega = MathUtil.applyDeadband(xOmegaSupplier.getAsDouble(), DEADBAND);
          double yOmega = MathUtil.applyDeadband(yOmegaSupplier.getAsDouble(), DEADBAND);
          double omega = 0;

          if (Math.abs(xOmega) + Math.abs(yOmega) > 0.1) {
            m_thetaController.setP(Constants9752.SwerveConstants.driveHeadingKp);
            m_thetaController.setD(Constants9752.SwerveConstants.driveHeadingKd);
            m_thetaController.setSetpoint(Math.toDegrees(Math.atan2(xOmega, yOmega)));
            omega =
                m_thetaController.calculate(
                    (MathUtil.inputModulus(drive.getRotation().getDegrees(), -180, 180)),
                    m_thetaController.getSetpoint());
          }

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          xOmega = Math.copySign(xOmega * xOmega, xOmega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          omega = MathUtil.clamp(omega, -1, 1);
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }
}
