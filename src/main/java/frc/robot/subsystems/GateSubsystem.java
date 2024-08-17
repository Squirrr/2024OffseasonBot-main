// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class GateSubsystem extends SubsystemBase {

  final Servo gateServo = new Servo(ShooterConstants.kServoPort);
  double desired_pos = 0;

  /** Creates a new GateSubsystem. */
  public GateSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    DogLog.log("Gate/Position", gateServo.getPosition());
    DogLog.log("Gate/Speed", gateServo.getSpeed());
    DogLog.log("Gate/Desired_Position", desired_pos);
  }

  public void open() {
    desired_pos = 0.6;
    gateServo.setPosition(0.6);
  }

  public void close() {
    desired_pos = 0;
    gateServo.setPosition(0);
  }

  public Command Open() {
    return run(
        () -> {
          open();
        });
  }

  public Command Close() {
    return run(
        () -> {
          close();
        });
  }
}
