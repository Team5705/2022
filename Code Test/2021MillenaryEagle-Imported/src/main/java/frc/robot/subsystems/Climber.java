/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final WPI_VictorSPX m1 = new WPI_VictorSPX(43);
  private final WPI_VictorSPX m2 = new WPI_VictorSPX(9);
  private final WPI_VictorSPX up = new WPI_VictorSPX(5);

  private final DifferentialDrive scale = new DifferentialDrive(m1, m2);

  public Climber() {

  }

  public void speedClimber(double speed) {
    up.set(ControlMode.PercentOutput ,speed);

  }

  public void speedRobotClimber(double speed) {
    scale.arcadeDrive(speed, 0);
  }

  @Override
  public void periodic() {
  }
}
