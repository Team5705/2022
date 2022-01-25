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

public class Distance extends CommandBase {
  private final Powertrain powertrain;
  private static PID pidDistance;

  private double distance;
  private double maxLimit;
  private double tolerance = 0.05; // 5 cm

  double kP = 1,
         kI = 0, 
         kD = 800, 
         bias = 0.32;

  /**
   * 7v7
   * 
   * @param powertrain       Subsistema motriz
   * @param distanceInMeters Distancia deseada en metros
   * @param kP               Proporcional
   * @param kI               Integral
   * @param kD               Derivativo
   */
  public Distance(Powertrain powertrain, double distanceInMeters, double kP, double kI, double kD, double bias) {
    this.powertrain = powertrain;
    this.distance = distanceInMeters;
    pidDistance = new PID(kP, kI, kD, distanceInMeters, bias, false);

    addRequirements(powertrain);
  }

  /**
   * Comando para acanzar la distancia deseada en metros. Realiza el cálculo de PID y terminará cuando la distancia
   * sea mayor o igual tomando en cuenta la tolerancia dada. los valores de PID en este apartado ya están declarados.
   * 
   * @param powertrain
   * @param distanceInMeters
   */
  public Distance(Powertrain powertrain, double distanceInMeters) {
    this.powertrain = powertrain;
    this.distance = distanceInMeters;
    this.maxLimit = 0.8;

    pidDistance = new PID(kP, kI, kD, distanceInMeters, bias, false);

    addRequirements(powertrain);
  }

  public Distance(Powertrain powertrain, double distanceInMeters, double maxLimit) {
    this.powertrain = powertrain;
    this.distance = distanceInMeters;
    this.maxLimit = maxLimit;

    pidDistance = new PID(kP, kI, kD, distanceInMeters, bias, false);

    addRequirements(powertrain);
  }

  @Override
  public void initialize() {
    powertrain.resetEncoders();
    powertrain.resetGyro();

  }

  @Override
  public void execute() {
    pidDistance.runPID(powertrain.getDistanceRight());

    double turn = (0 - powertrain.navAngle()) * 0.05;

    SmartDashboard.putNumber("PID", pidDistance.valuePID());
    SmartDashboard.putNumber("PIDFilter", filter(pidDistance.valuePID(), maxLimit));

    powertrain.arcadeDrive(filter(pidDistance.valuePID(), maxLimit), turn);

  }

  private double filter(double value, double maxLimit){
    if (value > maxLimit) 
      return maxLimit;

    else if ( value < -maxLimit )
      return -maxLimit;

    else 
      return value;
  }

  @Override
  public void end(boolean interrupted) {
    powertrain.resetGyro();
    powertrain.resetEncoders();
  }

  @Override
  public boolean isFinished() {
    return (powertrain.getDistanceRight() >= (distance - tolerance));
  }
}
