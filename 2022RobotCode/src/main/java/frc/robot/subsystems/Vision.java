// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTable table2 = NetworkTableInstance.getDefault().getTable("CameraPublisher").getSubTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry ta = table.getEntry("ta");
  private NetworkTableEntry tv = table.getEntry("tv");
  private NetworkTableEntry available = table2.getEntry("description");

  //public UsbCamera cam0 = CameraServer.startAutomaticCapture(0);

  //private boolean availableCamera = false;

  // Vision Tape Height
  private final double visionTapeHeightFt = 2.62;// m   //8 + 2.25/12; // 8 feet, 2.25 inches
  private final double upperHUBVisionTargetDiameter = Units.inchesToMeters(53.38) / 2; //.25 tolerance

  // Camera height and angle
  private final double cameraHeightMeters = 0.765; // m
  private final double cameraMountingAngle = 25; // degrees
  private final double mountingRadians = Math.toRadians(cameraMountingAngle); // a1, converted to radians

  // result of h2 - h1
  private double differenceOfHeights = visionTapeHeightFt - cameraHeightMeters;

  public Vision() {
    ledsOff();
		//cam0.setResolution(120, 160);

  }

  public boolean availableLimeLight(){
    String name = available.getString(null);
    
    try {
      if(name.length() > 0)
      return true;
    else 
      return false;  
    } catch (Exception e) {
      return false;
    } 
  }

  /**
   * 
   * @return Error en grados.
   */
  public double getX() {
    return tx.getDouble(0.0);
  }

  /**
   * 
   * @return Error en grados.
   */
  public double getY() {
    return ty.getDouble(0.0);
  }

  public double getArea() {
    return ta.getDouble(0.0);
  }

  public boolean availableTarget() {
    if (tv.getDouble(0.0) > 0.0)
      return true;
    else
      return false;
  }

  public void ledsOff() {
    table.getEntry("ledMode").setNumber(1);
  }

  public void ledsOn() {
    table.getEntry("ledMode").setNumber(3);
  }

  public void blinkLeds() {
    table.getEntry("ledMode").setNumber(2);
  }

  public void blinkLeds2() {
    table.getEntry("ledMode").setNumber(6);
  }

  public void ledsDefault() {
    table.getEntry("ledMode").setNumber(0);
  }

  public void visionProcessorMode() {
    table.getEntry("camMode").setNumber(0);
  }

  public void driverCameraMode() {
    table.getEntry("camMode").setNumber(1);
  }

  public void selectPipeline(int pipeline) {
    table.getEntry("pipeline").setNumber((double) pipeline);
  }

  /**
   * Devuelve la distancia en el eje horizontal.
   * @return La distancia en metros
   */
  public double getDistance(){

    // a2 to radians
    double radiansToTarget = Math.toRadians(ty.getDouble(0.0));
    //152cm con a2 = -0;.16

    // result of a1 + a2
    double angleInRadians = mountingRadians + radiansToTarget;

    // tangent of a1 + a1
    double tangentOfAngle = Math.tan(angleInRadians);

    double distance = differenceOfHeights/tangentOfAngle;

    if(availableTarget())
      return distance + upperHUBVisionTargetDiameter/2;
    else
      return 0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("tX", getX());
    SmartDashboard.putNumber("tY", getY());
    SmartDashboard.putNumber("Area", getArea());
    SmartDashboard.putNumber("Distance", getDistance());
    SmartDashboard.putBoolean("targetVisible", availableTarget());
  }
}
