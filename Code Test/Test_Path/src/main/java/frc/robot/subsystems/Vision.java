// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTable table2 = NetworkTableInstance.getDefault().getTable("CameraPublisher").getSubTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private static NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry ta = table.getEntry("ta");
  private static NetworkTableEntry tv = table.getEntry("tv");
  private NetworkTableEntry available = table2.getEntry("description");

  public static UsbCamera cam0 = CameraServer.startAutomaticCapture(0);

  //private boolean availableCamera = false;

  // Vision Tape Height
  private static final double visionTapeHeightFt = 2.62;// m   //8 + 2.25/12; // 8 feet, 2.25 inches

  // Camera height and angle
  private static final double cameraHeightInches = 0.76; // m
  private static final double cameraMountingAngle = 24; // degrees
  private static final double mountingRadians = Math.toRadians(cameraMountingAngle); // a1, converted to radians

  // result of h2 - h1
  private static double differenceOfHeights = visionTapeHeightFt - cameraHeightInches;

  public Vision() {
    ledsOff();
		cam0.setResolution(120, 160);

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

  public double getX() {
    return tx.getDouble(0.0);
  }

  public double getY() {
    return ty.getDouble(0.0);
  }

  public double getArea() {
    return ta.getDouble(0.0);
  }

  public static boolean availableTarget() {
    if (tv.getDouble(0.0) > 0)
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

  public static void ledsDefault() {
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
  public static double getDistance(){

    // a2 to radians
    double radiansToTarget = Math.toRadians(ty.getDouble(0.0));
    //152cm con a2 = -0;.16

    // result of a1 + a2
    double angleInRadians = mountingRadians + radiansToTarget;

    // tangent of a1 + a1
    double tangentOfAngle = Math.tan(angleInRadians);

    double distance = differenceOfHeights/tangentOfAngle;

    if(availableTarget())
      return distance;
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
