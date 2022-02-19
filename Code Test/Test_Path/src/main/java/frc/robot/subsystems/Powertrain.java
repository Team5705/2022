// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
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

  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(pathWeaver.kTrackwidthMeters);
  private Pose2d initialPosition = new Pose2d(0, 0, new Rotation2d(0));
  
  private final DifferentialDriveOdometry odometry;

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(pathWeaver.ksVolts, 
                                                                          pathWeaver.kvVoltSecondsPerMeter, 
                                                                          pathWeaver.kaVoltSecondsSquaredPerMeter);

  private PIDController leftPIDController = new PIDController(pathWeaver.kPDriveVel, 0, 0);
  private PIDController rightPIDController = new PIDController(pathWeaver.kPDriveVel, 0, 0);

  //private Pose2d pose;

  public Powertrain() {
    gyro.calibrate();

    configTalon_Victor();

    resetEncoders();
    zeroHeading();

    odometry = new DifferentialDriveOdometry(ahrs.getRotation2d(), initialPosition);

    new PrintCommand("Powertrain iniciado");
  }

  /**
   * Hola que hace
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

  private double last_world_linear_accel_x;
  private double last_world_linear_accel_y;

  final static double kCollisionThreshold_DeltaG = 0.5f;

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

  public double positionLeft() {
    return leftMaster.getSelectedSensorPosition(0);
  }

  public double positionRight() {
    return rightMaster.getSelectedSensorPosition(0);
  }

  /**
   * 7u7
   * 
   * @return La distancia en metros
   */
  public double getDistanceLeft() {
    double value = Units.inchesToMeters(((double) -positionLeft() / 4096) * (Math.PI * 6.00)); // Invertidow
    return value; // en sentido horario del robot marca negativo
  }

  /**
   * UwU
   * 
   * @return La distancia en metros
   */
  public double getDistanceRight() {
    double value = Units.inchesToMeters( ((double) positionRight() / 4096) * (Math.PI * 6.00) );
    return value;
  }

  /**
   * n_n
   * 
   * @return La velocidad en metros por segundo
   */
  public double getRateLeft() {
    return (((double) -leftMaster.getSelectedSensorVelocity(0) * 10) / 4096) * (Units.inchesToMeters(6) * Math.PI);
    //return -leftMaster.getSelectedSensorVelocity(0);
  }

  /**
   * :3
   * 
   * @return La velocidad en metros por segundo
   */
  public double getRateRight() {
    return (((double) rightMaster.getSelectedSensorVelocity(0) * 10) / 4096) * (Units.inchesToMeters(6) * Math.PI);
    //return rightMaster.getSelectedSensorVelocity(0);
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeedss() {
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
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
      drive.setMaxOutput(maxOutput);
  }

  public SimpleMotorFeedforward getFeedFoward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
      return leftPIDController;
  }

  public PIDController getRightPIDController() {
      return rightPIDController;
  }

  public DifferentialDriveKinematics getDifferentialDriveKinematics() {
      return kinematics;
  }

  public double aangle() {
    return gyro.getAngle();
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

  public static double ticksToMeters(double ticks) {
    return ticks * pathWeaver.TICKS_TO_METERS_RATIO;
}

  public void updateOdometry() {
     odometry.update(ahrs.getRotation2d(), getDistanceLeft(), getDistanceRight());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
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

    /* Dashboard.sendDouble("gyro", angleNormalized());
    Dashboard.sendDouble("rateL", getRateLeft());
    Dashboard.sendDouble("rateR", getRateRight());
    Dashboard.sendBoolean("navReady", !ahrs.isCalibrating()); */
  }

  private void configTalon_Victor() {
    leftMaster.configFactoryDefault();
    rightMaster.configFactoryDefault();
    leftFollow.configFactoryDefault();
    rightFollow.configFactoryDefault();

    leftFollow.follow(leftMaster);
    rightFollow.follow(rightMaster);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    leftFollow.setInverted(InvertType.FollowMaster);
    rightFollow.setInverted(InvertType.FollowMaster);

    try {
      leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
      leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1);

      rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
      rightMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1);
    } catch (Exception e) {
      new PrintCommand("Encoders unavailables!");
      System.out.println();
    }


  }


 


}

