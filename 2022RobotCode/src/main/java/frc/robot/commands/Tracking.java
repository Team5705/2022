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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PID;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Powertrain;
import frc.robot.subsystems.Vision;

public class Tracking extends CommandBase {
  private final Vision vision;
  private final Powertrain powertrain;
  private static PID pidX;
  private boolean finished = false;
  private double distance = 0;
  private final double range = 0.1;

  private final double minimumDistance = 2.70; //meters
  private final double maximumDistance = 4.60; //meterss

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("PID_Test");
  private NetworkTableEntry nkP = table.getEntry("kP");
  private NetworkTableEntry nkI = table.getEntry("kI");
  private NetworkTableEntry nkD = table.getEntry("kD");
  private NetworkTableEntry nkF = table.getEntry("kF");

  private double kP = 0.02, kI = 0.0, kD = 8.0, kF = 0.27;

  /**
   * Ejecuta el seguimiento al centro del objetivo sin un fin establecido.
   * 
   * @param powertrain Subsistema motriz
   * @param vision Subsistema de vision
   */
  public Tracking(Powertrain powertrain, Vision vision) {
    this.powertrain = powertrain;
    this.vision = vision;
    addRequirements(powertrain);

    pidX = new PID(0.02, 0, 8, 0, 0.27, false);
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
  public Tracking(Powertrain powertrain, Vision vision, boolean finished) {
    this.powertrain = powertrain;
    this.vision = vision;
    this.finished = finished;

    addRequirements(powertrain);

    pidX = new PID(0.02, 0, 8, 0, 0.27, false);
  }

  @Override
  public void initialize() {
    vision.ledsOn();
    vision.selectPipeline(0);//Pipeline donde el objetivo es el centro de la imagen

   }

  @Override
  public void execute() {
    update();
    pidX.setValues(kP, kI, kD, kF);

    //Ajustar la distancia para el tiro en el ultimo valor           | Aqui
    distance = vision.getDistance() + 0;
    double xS;

    if(distance < minimumDistance){
      //Distancia muy cercana!
      xS = 0.5;
    }
    else if(distance > maximumDistance){
      //Distancia muy lejana!
      xS = -0.5;
    }
    else{
      //Distancia no filtrada!
      xS = 0;
    }

    pidX.runPIDErr(vision.getX());

    if (vision.availableTarget()) {

      double turn = pidX.valuePID();

      powertrain.arcadeDrive(xS, turn);

    } 
    else{
      double speed = RobotContainer.driverController.getRawAxis(3)-RobotContainer.driverController.getRawAxis(2);
      powertrain.arcadeDrive(speed, RobotContainer.driverController.getRawAxis(0));

    }
  }

  @Override
  public void end(boolean interrupted) {
    powertrain.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {

    if (finished){

      return (Math.abs(vision.getX()) <= range && vision.getX() != 0) && (distance > minimumDistance && distance < maximumDistance);
    }
    else {
      return false;
    }
  }

  public void update(){
    kP = nkP.getDouble(0);
    kI = nkI.getDouble(0);
    kD = nkD.getDouble(0);
    kF = nkF.getDouble(0);

    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);
    SmartDashboard.putNumber("kF", kF);
  }
}
