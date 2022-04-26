/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PID;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Powertrain;
import frc.robot.subsystems.Vision;

public class SimpleTracking extends CommandBase {
  private final Vision vision;
  private final Powertrain powertrain;
  private static PID pidX = new PID(0.1, 0, 6, 0, 0.1, false);//0.16
  private static PID pidY = new PID(0.1, 0, 6, 0, 0.1, false);//0.15
  private boolean finished = false;
  private final double range = 0.9;


  private NetworkTable table = NetworkTableInstance.getDefault().getTable("PID_Test");
  private NetworkTableEntry nkP = table.getEntry("kP");
  private NetworkTableEntry nkI = table.getEntry("kI");
  private NetworkTableEntry nkD = table.getEntry("kD");
  private NetworkTableEntry nkF = table.getEntry("kF");

  private double kP = 0.0, kI = 0.0, kD = 0.0, kF = 0.0;

  /**
   * Ejecuta el seguimiento al centro del objetivo sin un fin establecido.
   * 
   * @param powertrain Subsistema motriz
   * @param vision Subsistema de vision
   */
  public SimpleTracking(Powertrain powertrain, Vision vision) {
    this.powertrain = powertrain;
    this.vision = vision;
    addRequirements(powertrain);
  }

  /**
   * Ejecuta el seguimiento cerca del centro del objetivo. Este modo es para que el comando funcione solo hasta que esté dentro
   * del rango establecido para luego pasar al siguiente comando.
   * 
   * @param powertrain Subsistema motriz
   * @param vision Subsistema de vision
   * @param finished Indica si queremos que el comando termine o no, de ser falso nunca terminará hasta que sea interrumpido,
   *                 de ser verdadero terminará cuando el robot esté alineado con el objetivo. Dejar en verdadero.
   */
  public SimpleTracking(Powertrain powertrain, Vision vision, boolean finished) {
    this.powertrain = powertrain;
    this.vision = vision;
    this.finished = finished;

    addRequirements(powertrain);
  }

  @Override
  public void initialize() {
    vision.ledsOn();
    vision.selectPipeline(0); //Pipeline calibrada para la distancia correcta

   }

  @Override
  public void execute() {
    //pidY.setValues(kP, kI, kD, kF);

    pidX.runPIDErr(vision.getX());
    pidY.runPIDErr(vision.getY());

    if (vision.availableTarget()) {

      double xS = pidY.valuePID();
      double turn = pidX.valuePID();

      powertrain.arcadeDrive(xS, turn);

    } 
    else{
      powertrain.arcadeDrive(
        RobotContainer.driverController.getRawAxis(3) - RobotContainer.driverController.getRawAxis(2), 
        RobotContainer.driverController.getRawAxis(0));

    }
  }

  @Override
  public void end(boolean interrupted) {
    powertrain.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {

    if (finished){

      return (Math.abs(vision.getX()) <= range && vision.getX() != 0) && (Math.abs(vision.getY()) <= range && vision.getY() != 0);
    }
    else {
      return false;
    }
  }

  public void update(){
  }
}
