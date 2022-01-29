// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.pt;

public class DRIVE extends CommandBase {
  private final pt chasis; 
  /** Creates a new DRIVE. */
  public DRIVE(pt chasis) {
    this.chasis = chasis; 
    addRequirements(chasis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = RobotContainer.drive.getRawAxis(1);
    double turn = RobotContainer.drive.getRawAxis(4);
    chasis.dt(speed, turn);
    SmartDashboard.putNumber("speed", speed);
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
