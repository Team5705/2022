// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kClimber;
import frc.robot.Constants.kGlobal;

public class Climber extends SubsystemBase {
  private final Solenoid left = new Solenoid(kGlobal.portPCM, PneumaticsModuleType.CTREPCM, kClimber.leftSolenoid);

  /** Creates a new Climber. */
  public Climber() {
  }

  public void extend(){
    left.set(true);
  }

  public void contract(){
    left.set(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Climber", left.get());
  }
}
