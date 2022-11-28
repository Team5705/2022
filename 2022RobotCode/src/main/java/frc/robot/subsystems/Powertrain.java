// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Powertrain extends SubsystemBase {
 /*  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(DriveConstant.portsMotors[0]),
                              rightMaster = new WPI_TalonSRX(DriveConstant.portsMotors[2]);
  private final WPI_VictorSPX leftFollow = new WPI_VictorSPX(DriveConstant.portsMotors[1]),
                              rightFollow = new WPI_VictorSPX(DriveConstant.portsMotors[3]); */

  private final CANSparkMax leftMaster = new CANSparkMax(kDrive.portsMotors[0], MotorType.kBrushless),
                            leftFollow = new CANSparkMax(kDrive.portsMotors[1], MotorType.kBrushless),
                            rightMaster = new CANSparkMax(kDrive.portsMotors[2], MotorType.kBrushless),
                            rightFollow = new CANSparkMax(kDrive.portsMotors[3], MotorType.kBrushless);
  
  private final WPI_CANCoder leftEncoder = new WPI_CANCoder(kDrive.leftEncoder),
                             rightEncoder = new WPI_CANCoder(kDrive.rightEncoder);
  private CANCoderConfiguration leftEncoderConfigs = new CANCoderConfiguration(),
                                rightEncoderConfigs = new CANCoderConfiguration();

  private final DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);
  
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(pathWeaver.kTrackwidthMeters);

  private final AHRS ahrs = new AHRS(Port.kMXP);

  private Pose2d initialPosition = new Pose2d(pathWeaver.xInitialPosition, // x
                                              pathWeaver.yInitialPosition, // y
                                              Rotation2d.fromDegrees(pathWeaver.initialDegree)); // Heading in degrees
  private final DifferentialDriveOdometry odometry;

  private final Field2d field = new Field2d();

  private double last_world_linear_accel_x;
  private double last_world_linear_accel_y;
  final static double kCollisionThreshold_DeltaG = 0.5f;

  public Powertrain() {

    configControllers();
    configEncoders();

    resetEncoders();
    zeroHeading();
    ahrs.calibrate();
    //ahrs.reset();
    //ahrs.setAngleAdjustment(pathWeaver.initialDegree);

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(navAngle()), initialPosition);
    SmartDashboard.putData("Field", field);


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

  public double [] arcadeDriveMetersPerSeconds(double linear, double angular){
    var wheelSpeeds = new DifferentialDriveWheelSpeeds(3.5*linear, 3.5*angular);

    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);

    double linearVelocity = chassisSpeeds.vxMetersPerSecond;
    double angularVelocity = chassisSpeeds.omegaRadiansPerSecond;

    return new double [] {linearVelocity, angularVelocity};
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void setVolts(double leftVolts, double rightVolts) {

    leftMaster.setVoltage(leftVolts);//*(10/12));
    rightMaster.setVoltage(rightVolts);//*(10/12));
    drive.feed();

  }

  

  public double positionLeft() {
    return leftEncoder.getPosition();
  }

  public double positionRight() {
    return rightEncoder.getPosition();
  }

  /**
   * 7u7<p>
   * Distancia del lado izquierdo del chasis
   * 
   * @return La distancia en metros
   */
  public double getDistanceLeft() {
    //double value = Units.inchesToMeters( ((double) positionLeft() / 4096) * (Math.PI * 6.00)); // Invertidow
    return leftEncoder.getPosition(); // en sentido horario del robot marca negativo
  }

  /**
   * UwU<p>
   * Distancia del lado derecho del chasis
   * 
   * @return La distancia en metros
   */
  public double getDistanceRight() {
    //double value = (positionRight() / 4096) * (Math.PI * Units.inchesToMeters(6.00));
    return rightEncoder.getPosition();
  }

  /**
   * n_n<p>
   * Obtiene la velocidad del lado izquierdo
   * 
   * @return La velocidad en metros por segundo
   */
  public double getRateLeft() {
    //return (((double) leftMaster.getSelectedSensorVelocity(0) * 10) / 4096) * (Units.inchesToMeters(6) * Math.PI);
    return leftEncoder.getVelocity();
  }

  /**
   * :3<p>
   * Obtiene la velocidad del lado derecho
   * 
   * @return La velocidad en metros por segundo
   */
  public double getRateRight() {
    //return (rightMaster.getSelectedSensorVelocity(0) / 4096) * (Units.inchesToMeters(6) * Math.PI) * 10;
    return rightEncoder.getVelocity();
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
  /* public double getHeading() {
    return ahrs.getRotation2d().getDegrees();
    //return Rotation2d.fromDegrees(Math.IEEEremainder(navAngle(), 360) * (pathWeaver.kGyroReversed ? -1.0 : 1.0));
  } */

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
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
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
    leftMaster.setIdleMode(IdleMode.kBrake);
    leftMaster.setIdleMode(IdleMode.kBrake);
    leftFollow.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);
    rightFollow.setIdleMode(IdleMode.kBrake);
  }

  public void neutralModeCoast(){
    leftMaster.setIdleMode(IdleMode.kCoast);
    leftFollow.setIdleMode(IdleMode.kCoast);
    rightMaster.setIdleMode(IdleMode.kCoast);
    rightFollow.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    odometry.update(ahrs.getRotation2d(), getDistanceLeft(), getDistanceRight());
    field.setRobotPose(odometry.getPoseMeters());

    SmartDashboard.putNumber("gyro", angleNormalized());
    //SmartDashboard.putNumber("NavAngle", navAngle());
    SmartDashboard.putNumber("encoder_L", getDistanceLeft());
    SmartDashboard.putNumber("encoder_R", getDistanceRight());
    SmartDashboard.putNumber("positionL", positionLeft());
    SmartDashboard.putNumber("positionR", positionRight());
    SmartDashboard.putNumber("rateL", getRateLeft());
    SmartDashboard.putNumber("rateR", getRateRight());
    SmartDashboard.putBoolean("navX-MXP_Calibrated", !ahrs.isCalibrating());

    SmartDashboard.putNumber("X_Field", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y_Field", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Radians", odometry.getPoseMeters().getRotation().getRadians());
    SmartDashboard.putNumber("Degrees", odometry.getPoseMeters().getRotation().getDegrees());
  }

  private void configControllers() {
    //Configuraciones por defecto, reseteo de los controladores
    leftMaster.restoreFactoryDefaults();
    rightMaster.restoreFactoryDefaults();
    leftFollow.restoreFactoryDefaults();
    rightFollow.restoreFactoryDefaults();

    //Control de curva de aceleracion
    /* double kRamp = 0.0;//0.15;
    leftMaster.setOpenLoopRampRate(kRamp);
    leftFollow.setOpenLoopRampRate(kRamp);
    rightMaster.setOpenLoopRampRate(kRamp);
    rightFollow.setOpenLoopRampRate(kRamp);

    leftMaster.setClosedLoopRampRate(kRamp);
    leftFollow.setClosedLoopRampRate(kRamp);
    rightMaster.setClosedLoopRampRate(kRamp);
    rightFollow.setClosedLoopRampRate(kRamp); */

    leftFollow.follow(leftMaster);
    rightFollow.follow(rightMaster);

    boolean leftInverted = false;
    boolean rightInverted = true;

    leftMaster.setInverted(leftInverted);
    rightMaster.setInverted(rightInverted);
    
    leftFollow.setInverted(leftInverted);
    rightFollow.setInverted(rightInverted);
    
    neutralModeBrake();
  }

  private void configEncoders(){
    leftEncoderConfigs.sensorDirection = true; //Dirección del valor, ajustar si está invertido
    //Coeficiente para la salida de los valores, por defecto en grados (0.087890625) e.g. 4096 * 0.087890625 = 360°
    leftEncoderConfigs.sensorCoefficient = 1.1688933603688586170451827431929e-4; //Valores en metros
    leftEncoder.configAllSettings(leftEncoderConfigs);

    rightEncoderConfigs.sensorDirection = false;
    rightEncoderConfigs.sensorCoefficient = 1.1688933603688586170451827431929e-4; 
    rightEncoder.configAllSettings(rightEncoderConfigs);
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

