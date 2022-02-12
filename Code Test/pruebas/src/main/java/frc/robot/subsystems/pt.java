// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstant;

public class pt extends SubsystemBase {
  Spark m_frontleft = new Spark(DriveConstant.portsMotors[0]);
  Spark m_rearleft = new Spark(DriveConstant.portsMotors[1]);
  
  Spark m_frontright = new Spark (DriveConstant.portsMotors[2]);
  Spark m_rearRight = new Spark(DriveConstant.portsMotors[3]);

  MotorControllerGroup m_left = new MotorControllerGroup(m_frontleft, m_rearleft);
  MotorControllerGroup m_right = new MotorControllerGroup(m_frontright, m_rearRight);

  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

 // private final AHRS ahrs = new AHRS(Port.kMXP);

  /** Creates a new pt. */
  public pt() {
    m_left.setInverted(false);
    m_right.setInverted(true);
  }

 // public double navAngle() {
   // return ahrs.getAngle();
 // }

  public void resetGyro() {
 //   ahrs.reset();
  }

  public void dt(double speed, double turn){
  m_drive.arcadeDrive(speed, turn);
    
  } 

  @Override
  public void periodic() {
   // SmartDashboard.putNumber("NavAngle", navAngle());
  }
}
