// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class RobotConfig {
    public static String rioSerial = "empty";
    String ITKAN9128SERIAL2024 = "PUTSERIALHERE";
    String ITKAN9752SERIAL2024 = "PUTSERIALHERE";
    public static RobotType robotType = null;

    public static Constants botConstants;
    
    public RobotConfig() {
        if (Robot.isReal()) {
            Timer.delay(2.0); // Wait for the robot to fully boot up
        }
        // Set the RoboRio Serial number for this robot, useful for adjusting comp/practice bot
        // settings
        if (RobotController.getSerialNumber() != null) {
            rioSerial = RobotController.getSerialNumber();
            System.out.println("RIO SERIAL: " + rioSerial);
        }   
        
        if (Robot.isSimulation()) {
            robotType = RobotType.SIM;
        }if (rioSerial==ITKAN9128SERIAL2024) {
            botConstants = new Constants9128();
        } else if (rioSerial==ITKAN9752SERIAL2024) {
            botConstants = new Constants9752();
        } else {
            System.out.println("No valid serial found");
        }
    }

    public static RobotType getRobotType() {return robotType;}
    
    public enum RobotType {
        ITKANSR,
        ITKANJR,
        SIM
    }
}
