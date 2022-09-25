// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Powertrain;

public class Drive extends CommandBase {
  private final Powertrain powertrain;/* 
  private final Hood hood;
  private final Vision vision; */

  PIDController vController = new PIDController(0.6, //P
                                                0,    //I
                                                0);   //D

  SimpleMotorFeedforward aa;
  
  public Drive(Powertrain powertrain){//, Hood hood, Vision vision) {
    this.powertrain = powertrain;
   /*  this.hood = hood;
    this.vision = vision; */
    addRequirements(powertrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    powertrain.resetEncoders();
    vController.enableContinuousInput(-1, 1);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSp = RobotContainer.driverController.getRawAxis(3) - RobotContainer.driverController.getRawAxis(2);
    double turn = RobotContainer.driverController.getRawAxis(0);
    powertrain.arcadeDrive(xSp*1.0, turn * 1.0);
    
    SmartDashboard.putNumber("speed", xSp);
    SmartDashboard.putNumber("turn", turn);
    
    /* if (xSp > 0.95){
      new PrintCommand("Comando ejecutado");
    } */

    double pepe = vController.calculate(powertrain.getRateLeft(), xSp * 3.0);
    //powertrain.arcadeDrive(pepe, 0);
    SmartDashboard.putNumber("PIDVEL", pepe);
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
