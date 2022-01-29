// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PID;
import frc.robot.subsystems.pt;

public class Turn extends CommandBase {
  private final pt chasis;

  private static PID pidTurn;

  private double gyro;
  private double angle;

  double kP = 0.02, kI = 0.00000, kD = 11, bias = 0.4;

  /** Creates a new Turn. */
  public Turn(pt chasis, double angle) {
    this.chasis = chasis;
    this.angle = angle;
    addRequirements(chasis);

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidTurn = new PID(kP, kI, kD, angle, bias, false);
    
    chasis.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //  gyro = chasis.navAngle();
    
    pidTurn.runPID(gyro);

    double turn = pidTurn.valuePID();

    /*if (turn > 0.8) turn = 0.8;
    else if (turn < -0.8) turn = -0.8;
    else turn = pidTurn.valuePID();*/

    SmartDashboard.putNumber("PID", turn);
    chasis.dt(0, turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chasis.dt(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//(int) chasis.navAngle() == angle;
  }
}
