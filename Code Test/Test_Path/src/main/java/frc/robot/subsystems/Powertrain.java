// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstant;
import frc.robot.Constants.pathWeaver;

public class Powertrain extends SubsystemBase {
  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(DriveConstant.portsMotors[0]),
                             rightMaster = new WPI_TalonSRX(DriveConstant.portsMotors[2]);
  private final WPI_VictorSPX leftFollow = new WPI_VictorSPX(DriveConstant.portsMotors[1]),
                              rightFollow = new WPI_VictorSPX(DriveConstant.portsMotors[3]);
                              

  private final DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

  private final Gyro gyro = new ADXRS450_Gyro(DriveConstant.Gyro);
  private final AHRS ahrs = new AHRS(Port.kMXP);

  private Pose2d initialPosition = new Pose2d(pathWeaver.xInitialPosition, // x
                                              pathWeaver.yInitialPosition, // y
                                              Rotation2d.fromDegrees(pathWeaver.initialHeading)); // Heading in degrees
  private final DifferentialDriveOdometry odometry;

  private double last_world_linear_accel_x;
  private double last_world_linear_accel_y;
  final static double kCollisionThreshold_DeltaG = 0.5f;

  public Powertrain() {
    gyro.calibrate();

    configTalon_Victor();

    resetEncoders();
    zeroHeading();

    odometry = new DifferentialDriveOdometry(ahrs.getRotation2d(), initialPosition);

    new PrintCommand("Powertrain iniciado");
  }

  /**
   * Hace funcionar el chasis con valores de -1.0 a 1.0
   *
   * @param xSp  Velocidad en X
   * @param turn Velocidad en Y
   */
  public void arcadeDrive(double xSp, double turn) {
    drive.arcadeDrive(xSp, turn);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void setVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(rightVolts);
    drive.feed();

  }

  

  public double positionLeft() {
    return leftMaster.getSelectedSensorPosition(0);
  }

  public double positionRight() {
    return rightMaster.getSelectedSensorPosition(0);
  }

  /**
   * 7u7<p>
   * Distancia del lado izquierdo del chasis
   * 
   * @return La distancia en metros
   */
  public double getDistanceLeft() {
    double value = Units.inchesToMeters( ((double) positionLeft() / 4096) * (Math.PI * 6.00)); // Invertidow
    return value; // en sentido horario del robot marca negativo
  }

  /**
   * UwU<p>
   * Distancia del lado derecho del chasis
   * 
   * @return La distancia en metros
   */
  public double getDistanceRight() {
    double value = (positionRight() / 4096) * (Math.PI * Units.inchesToMeters(6.00));
    return value;
  }

  /**
   * n_n<p>
   * Obtiene la velocidad del lado izquierdo
   * 
   * @return La velocidad en metros por segundo
   */
  public double getRateLeft() {
    return (((double) leftMaster.getSelectedSensorVelocity(0) * 10) / 4096) * (Units.inchesToMeters(6) * Math.PI);
  }

  /**
   * :3<p>
   * Obtiene la velocidad del lado derecho
   * 
   * @return La velocidad en metros por segundo
   */
  public double getRateRight() {
    return (rightMaster.getSelectedSensorVelocity(0) / 4096) * (Units.inchesToMeters(6) * Math.PI) * 10;
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    //La velocidad tiene que estar en metros por segundo
    return new DifferentialDriveWheelSpeeds(getRateLeft(), getRateRight());
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return ahrs.getRotation2d().getDegrees();
    //return Rotation2d.fromDegrees(Math.IEEEremainder(navAngle(), 360) * (pathWeaver.kGyroReversed ? -1.0 : 1.0));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, ahrs.getRotation2d());
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    ahrs.reset();
    ahrs.zeroYaw();
  }

  /**
   * T_T
   * 
   * @return Devuelve el angulo del robot normalizado en un rango de 0 a 359
   */
  public double angleNormalized() {
    double operation;

    if (navAngle() >= 0)
      operation = navAngle() % 360;
    else
      operation = 360 - Math.abs(navAngle() % 360);

    return operation;
  }

  public double navAngle() {
    return ahrs.getAngle();
  }

  public void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0);

    rightMaster.setSelectedSensorPosition(0);
  }

  public void resetGyro() {
    ahrs.reset();
  }
  
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void neutralModeBrake(){
      leftMaster.setNeutralMode(NeutralMode.Brake);
      leftFollow.setNeutralMode(NeutralMode.Brake);
      rightMaster.setNeutralMode(NeutralMode.Brake);
      rightFollow.setNeutralMode(NeutralMode.Brake);
  }

  public void neutralModeCoast(){
    leftMaster.setNeutralMode(NeutralMode.Coast);
    leftFollow.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Coast);
    rightFollow.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    odometry.update(ahrs.getRotation2d(), getDistanceLeft(), getDistanceRight());

    SmartDashboard.putNumber("Angle Normalized", angleNormalized());
    SmartDashboard.putNumber("NavAngle", navAngle());
    SmartDashboard.putNumber("encoder_L", getDistanceLeft());
    SmartDashboard.putNumber("encoder_R", getDistanceRight());
    SmartDashboard.putNumber("positionL", positionLeft());
    SmartDashboard.putNumber("positionR", positionRight());
    SmartDashboard.putNumber("rateL", getRateLeft());
    SmartDashboard.putNumber("rateR", getRateRight());
    SmartDashboard.putBoolean("navX-MXP_Calibrated", !ahrs.isCalibrating());
  }

  private void configTalon_Victor() {
    //Configuraciones por defecto, reseteo de los controladores
    leftMaster.configFactoryDefault();
    rightMaster.configFactoryDefault();
    leftFollow.configFactoryDefault();
    rightFollow.configFactoryDefault();
    //Control de curva de acekeracion
    leftMaster.configOpenloopRamp(0.5);
    leftFollow.configOpenloopRamp(0.5);
    rightMaster.configOpenloopRamp(0.5);
    rightFollow.configOpenloopRamp(0.5);

    //Primero llamar a los encoders antes del setInverted!
    try {
      leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
      leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1);

      rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
      rightMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1);

      leftMaster.setSensorPhase(false);//CHECAR
      rightMaster.setSensorPhase(false);//CHECAR

      System.out.println("Encoders good!");
    } catch (Exception e) {
      new PrintCommand("Encoders unavailables!");
      System.out.println("Encoders unavailables!");
    }

    leftFollow.follow(leftMaster);
    rightFollow.follow(rightMaster);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);
    
    leftFollow.setInverted(InvertType.FollowMaster);
    rightFollow.setInverted(InvertType.FollowMaster);
    
    neutralModeBrake();
  }

  public boolean collision(){
    boolean collisionDetected = false;
          
    double curr_world_linear_accel_x = ahrs.getWorldLinearAccelX();
    double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;

    last_world_linear_accel_x = curr_world_linear_accel_x;

    double curr_world_linear_accel_y = ahrs.getWorldLinearAccelY();
    double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
    
    last_world_linear_accel_y = curr_world_linear_accel_y;
    
    if ( ( Math.abs(currentJerkX) > kCollisionThreshold_DeltaG ) ||
         ( Math.abs(currentJerkY) > kCollisionThreshold_DeltaG) ) {
        collisionDetected = true;
    }
    SmartDashboard.putBoolean("CollisionDetected", collisionDetected);
    return collisionDetected;
  }


 


}

