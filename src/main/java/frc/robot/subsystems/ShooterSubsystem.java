// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants9752.ShooterConstants;
import frc.robot.Constants9752.ShooterConstants.ShooterPIDConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  final CANSparkFlex rightShooterMotor =
      new CANSparkFlex(ShooterConstants.kRightShooterMotorPort, MotorType.kBrushless);

  final CANSparkFlex leftShooterMotor =
      new CANSparkFlex(ShooterConstants.kLeftShooterMotorPort, MotorType.kBrushless);

  final RelativeEncoder rightShooterEncoder = rightShooterMotor.getEncoder();
  final RelativeEncoder leftShooterEncoder = leftShooterMotor.getEncoder();

  final SparkPIDController rightPIDController = rightShooterMotor.getPIDController();
  final SparkPIDController leftPIDController = leftShooterMotor.getPIDController();

  public boolean shooterSpeedReached = false;
  double desiredLeftShooterRPM, desiredRightShooterRPM = 0;

  public ShooterSubsystem() {
    rightShooterMotor.restoreFactoryDefaults();
    leftShooterMotor.restoreFactoryDefaults();

    rightShooterMotor.setInverted(true);

    rightPIDController.setFF(ShooterPIDConstants.kFF);
    rightPIDController.setP(ShooterPIDConstants.kP);
    rightPIDController.setI(ShooterPIDConstants.kI);
    rightPIDController.setD(ShooterPIDConstants.kD);

    leftPIDController.setFF(ShooterPIDConstants.kFF);
    leftPIDController.setP(ShooterPIDConstants.kP);
    leftPIDController.setI(ShooterPIDConstants.kI);
    leftPIDController.setD(ShooterPIDConstants.kD);

    rightShooterMotor.burnFlash();
    leftShooterMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    DogLog.log("Shooter/LeftShooter/Speed(RPM)", leftShooterEncoder.getVelocity());
    DogLog.log("Shooter/RightShooter/Speed(RPM)", rightShooterEncoder.getVelocity());
    DogLog.log("Shooter/LeftShooter/Desired_Speed(RPM)", desiredLeftShooterRPM);
    DogLog.log("Shooter/RightShooter/Desired_Speed(RPM)", desiredRightShooterRPM);
  }

  public Command DefaultCommand() {
    return run(
        () -> {
          leftShooterMotor.stopMotor();
          rightShooterMotor.stopMotor();
        });
  }

  public void setShooterRPM(double leftFlywheelSetPoint, double rightFlywheelSetPoint) {
    desiredLeftShooterRPM = leftFlywheelSetPoint;
    desiredRightShooterRPM = rightFlywheelSetPoint;
    shooterSpeedReached = false;
    leftPIDController.setReference(leftFlywheelSetPoint, ControlType.kVelocity);
    rightPIDController.setReference(rightFlywheelSetPoint, ControlType.kVelocity);

    if (rightShooterEncoder.getVelocity() > rightFlywheelSetPoint * 0.9
        && rightShooterEncoder.getVelocity() < rightFlywheelSetPoint * 1.1) {
      shooterSpeedReached = true;
    } else {
      shooterSpeedReached = false;
    }
  }

  public double[] currentShooterSpeed() {
    double[] shooterVelocities = {
      leftShooterEncoder.getVelocity(), rightShooterEncoder.getVelocity()
    };
    return shooterVelocities;
  }

  public Command SpinShooters(double leftFlywheelRPM, double rightFlywheelRPM) {
    return run(
        () -> {
          setShooterRPM(leftFlywheelRPM, rightFlywheelRPM);
        });
  }
}
