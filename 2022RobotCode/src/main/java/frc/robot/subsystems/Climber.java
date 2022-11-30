// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kClimber;
import frc.robot.Constants.kGlobal;

public class Climber extends SubsystemBase {
  //private final Solenoid left = new Solenoid(kGlobal.portPCM, PneumaticsModuleType.CTREPCM, kClimber.leftSolenoid);
  private DoubleSolenoid sol;

  /** Creates a new Climber. */
  public Climber() { 
    sol = new DoubleSolenoid(kGlobal.portPCM, PneumaticsModuleType.CTREPCM, kClimber.leftSolenoid, kClimber.rightSolenoid);
  }

  public void extend(){
    sol.set(Value.kForward);
  }

  public void contract(){
    sol.set(Value.kReverse);
  }

  public void off(){
    sol.set(Value.kOff);
  }

  public void periodic() {}
}
