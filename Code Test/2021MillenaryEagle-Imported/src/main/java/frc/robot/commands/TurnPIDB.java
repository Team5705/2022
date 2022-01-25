/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PID;
import frc.robot.subsystems.Powertrain;

public class TurnPIDB extends CommandBase {
  private final Powertrain powertrain;

  private static PID pidTurn;

  private double gyro;
  private double angle;
  private boolean direction = false; // true = clockwise | false = counter-clockwise

  double kP = 0.008, kI = 0.000000, kD = 5, bias = 0.42;//0.18;

  /**
   * unu
   * 
   * @param powertrain     Subsistema motriz
   * @param angle          Angulo deseado teniendo en cuenta que el angulo se
   *                       resetea y un giro inverso es necesario el angulo
   *                       negativo
   */
  public TurnPIDB(Powertrain powertrain, int angle, double kP, double kI, double kD) {
    this.powertrain = powertrain;
    this.angle = angle;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    addRequirements(this.powertrain);
  }

  /**
   * unu
   * 
   * Si.
   * 
   * @param powertrain Subsistema motriz
   * @param angle Angulo deseado entre los 360 grados del robot.
   */
  public TurnPIDB(Powertrain powertrain, int angle, boolean direction) {
    this.powertrain = powertrain;
    this.angle = angle;
    this.direction = direction;

    addRequirements(this.powertrain);
  }

  public TurnPIDB(Powertrain powertrain, int angle) {
    this.powertrain = powertrain;
    this.angle = angle;

    addRequirements(this.powertrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidTurn = new PID(kP, kI, kD, angle, bias, direction);

    powertrain.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    gyro = powertrain.navAngle();
    
    pidTurn.runPID(gyro);

    double turn = pidTurn.valuePID();

    SmartDashboard.putNumber("PID", turn);
    SmartDashboard.putNumber("PIDFilter", filter(turn));

    powertrain.arcadeDrive(0, filter(turn));

  }

  public double filter(double value){
    if (value > 0.9) 
      return 0.9;
    else if ( value < -0.9 )
      return -0.9;
    else return value;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (powertrain.navAngle()) >= (angle - 1.0) && powertrain.navAngle() < (angle + 1.0);
  }
}
