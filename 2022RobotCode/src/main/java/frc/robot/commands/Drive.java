// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.ShooterCommands.AdjustHood;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Powertrain;
import frc.robot.subsystems.Vision;

public class Drive extends CommandBase {
  private final Powertrain powertrain;/* 
  private final Hood hood;
  private final Vision vision; */
  
  public Drive(Powertrain powertrain){//, Hood hood, Vision vision) {
    this.powertrain = powertrain;
   /*  this.hood = hood;
    this.vision = vision; */
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
    powertrain.arcadeDrive(xSp*0.95, turn * 0.75);

    SmartDashboard.putNumber("speed", xSp);
    SmartDashboard.putNumber("turn", turn);
    /*  if (xSp > 1.0){
       new AdjustHood(hood, vision);
     } */
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
