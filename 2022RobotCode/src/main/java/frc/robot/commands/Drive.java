// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Powertrain;

public class Drive extends CommandBase {
  private final Powertrain powertrain;
  
  public Drive(Powertrain powertrain) {
    this.powertrain = powertrain;
    addRequirements(this.powertrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    powertrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSp = RobotContainer.driverController.getRawAxis(3) - RobotContainer.driverController.getRawAxis(2);
    double turn = RobotContainer.driverController.getRawAxis(0);
    powertrain.arcadeDrive(xSp*0.8, turn * 0.5);

    SmartDashboard.putNumber("speed", xSp);
    SmartDashboard.putNumber("turn", turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
