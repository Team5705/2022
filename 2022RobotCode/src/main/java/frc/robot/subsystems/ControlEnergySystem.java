// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class ControlEnergySystem extends SubsystemBase {
  private final PneumaticsControlModule pcm = new PneumaticsControlModule(kGlobal.portPCM);
  private final PowerDistribution pdp = new PowerDistribution(kGlobal.portPDP, ModuleType.kCTRE);

  /**
    powerFrontLeft: 12,
    powerFrontRight: 0,
    powerBackLeft: 5,
    powerBackRight: 0,

    powerIntake: 20,
    powerConveyor1: 0,
    powerConveyor2: 0,
    powerShooter1: 0,
    powerShooter2: 0,
    powerHood1: 0,
    powerHood2: 0,

    powerRio: 0,
    powerPcm: 0,
    batteryVoltage: 12,
    totalPowerUse: 15,
    rioCpu: 60,
    rioRam: 75,
   */
  public ControlEnergySystem() {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("powerFrontLeft", pdp.getCurrent(kDrive.channelsMotors[0]));
    SmartDashboard.putNumber("powerFrontright", pdp.getCurrent(kDrive.channelsMotors[1]));
    SmartDashboard.putNumber("powerBackLeft", pdp.getCurrent(kDrive.channelsMotors[2]));
    SmartDashboard.putNumber("powerBackRight", pdp.getCurrent(kDrive.channelsMotors[3]));

    SmartDashboard.putNumber("powerIntake", pdp.getCurrent(kIntake.channel));
    SmartDashboard.putNumber("powerConveyor1", pdp.getCurrent(kConveyor.channel_mSUPP));
    SmartDashboard.putNumber("powerConveyor2", pdp.getCurrent(kConveyor.channel_mADC));
    SmartDashboard.putNumber("powerShooter1", pdp.getCurrent(kShooter.channel_m1));
    SmartDashboard.putNumber("powerShooter2", pdp.getCurrent(kShooter.channel_m2));
    SmartDashboard.putNumber("powerHood1", pdp.getCurrent(kShooter.channel_hood));

    SmartDashboard.putNumber("powerRio", RobotController.getInputVoltage());
    SmartDashboard.putNumber("powerRioCurrent", RobotController.getInputCurrent());
    SmartDashboard.putNumber("powerPcm", pcm.getCompressorCurrent());
    SmartDashboard.putNumber("batteryVoltage", pdp.getVoltage());
    SmartDashboard.putNumber("totalPowerUse", pdp.getTotalCurrent());
    //SmartDashboard.putNumber("rioCpu", );
    SmartDashboard.putNumber("rioRam", (double) Runtime.getRuntime().freeMemory()/1024/1000);
  }
}
