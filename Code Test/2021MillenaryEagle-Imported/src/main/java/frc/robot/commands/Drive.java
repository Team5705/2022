/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Powertrain;

public class Drive extends CommandBase {
  private final Powertrain powertrain;

  /**
   * Creates a new Drive.
   */
  public Drive(Powertrain powertrain) {
    this.powertrain = powertrain;
    addRequirements(this.powertrain);
  }

  @Override
  public void initialize() {
    powertrain.resetEncoders();
  }

  @Override
  public void execute() {
    double xSp = RobotContainer.driverController.getRawAxis(3) - RobotContainer.driverController.getRawAxis(2);
    double turn = RobotContainer.driverController.getRawAxis(0);
    powertrain.arcadeDrive(xSp, turn * 0.7);

    SmartDashboard.putNumber("speed", xSp);
    SmartDashboard.putNumber("turn", turn);

    /*if(powertrain.collision()) {
      RobotContainer.driverController.setRumble(RumbleType.kLeftRumble, 0.7);
      RobotContainer.driverController.setRumble(RumbleType.kRightRumble, 0.7);
    }*/

    // Siempre tomen chavos Ctrl+K plus Ctrl+T ^w^/ 
    // Un antes y un despues <3
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
