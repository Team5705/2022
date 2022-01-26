/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.OIConstant;
import frc.robot.Constants.pathWeaver;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Powertrain powertrain = new Powertrain();
  private final Vision vision = new Vision();
  private final IntakeBalls intake = new IntakeBalls();
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();
  public static Leds leds = new Leds();

  private final Drive drive = new Drive(powertrain);
  private final Climberr climberr = new Climberr(climber);

  public static XboxController driverController = new XboxController(OIConstant.controllerPort);

  Trajectory trajectory;

  SendableChooser<String> autonomous = new SendableChooser<String>();
  
  ArrayList<String> trajectoryPaths = new ArrayList<String>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    
    powertrain.setDefaultCommand(drive);
    climber.setDefaultCommand(climberr);
    
    autonomous.addOption("Trench", "trench");
    autonomous.addOption("Mid", "mid");
    autonomous.addOption("Simple", "simple");
    autonomous.addOption("Emergency", "emergency");
    autonomous.addOption("Test", "test");
    SmartDashboard.putData("autoMode", autonomous);
    
    trajectoryPaths.add(0, "paths/output/a.wpilib.json");
    trajectoryPaths.add(1, "paths/YourPath.wpilib.json"); //EJemplo
    
    configureButtonBindings();

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryPaths.get(0));
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryPaths, ex.getStackTrace());
        
      } 
  }

  /**
   * 
   * @param trajectoryPaths Número del path en la lista declarada
   * @return  Comando ramsete para su utilización en el seguimiento del path
   */
  /*public Command ramseteC(String trajectoryPaths) {
    String path = trajectoryPaths;
    Trajectory trajectory;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      
      System.out.println("Path successfully read");
      new PrintCommand("Path successfully read");

    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryPaths, ex.getStackTrace());
        System.out.println("Unable to open trajectory: " + path);
        new PrintCommand("Unable to open trajectory: " + path);
        
      } 
    
      return new RamseteCommand(trajectory, powertrain::getPosition,
                new RamseteController(2.0, .7), powertrain.getFeedFoward(), powertrain.getDifferentialDriveKinematics(),
                powertrain::getWheelSpeeds, powertrain.getLeftPIDController(), powertrain.getRightPIDController(),
                powertrain::setVolts, powertrain);
                powertrain.zeroHeading();

  }*/
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Buttons
    //new JoystickButton(driverController, 1).whenActive(ramseteC("paths/output/a.wpilib.json"));
    new JoystickButton(driverController, 2).whenPressed(new TurnPIDB(powertrain, 50));
    new JoystickButton(driverController, 3).whenPressed(new TurnPIDB(powertrain, 110));
    //new JoystickButton(driverController, 2).whenPressed(new InstantCommand(intake::toExtendIntake, intake));
    //new JoystickButton(driverController, 3).whenPressed(new InstantCommand(intake::saveIntake, intake));
    //new JoystickButton(driverController, 4).whenPressed(new Shoot(shooter, intake, powertrain, vision));
    new JoystickButton(driverController, 4).whileHeld(new Tracking(powertrain, vision));
    new JoystickButton(driverController, 5).whileHeld(new TakeAll(intake));
    new JoystickButton(driverController, 6).toggleWhenPressed(new TakeWithSensor(intake));
    //new JoystickButton(driverController, 6).toggleWhenPressed(new TakeWithSensor(intake));

    //new JoystickButton(driverController, 8).whenPressed(new InstantCommand(powertrain::resetGyro)); // Boton Select

    //POV
    new POVButton(driverController, 270).whileHeld(new EjectBalls(intake));
  }

  //-----------------------------------------------------------------------------------------------------------------------------------------

  public Command getAutonomousCommand(){
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                pathWeaver.ksVolts,
                pathWeaver.kvVoltSecondsPerMeter,
                pathWeaver.kaVoltSecondsSquaredPerMeter),
            pathWeaver.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                pathWeaver.kMaxSpeedMetersPerSecond,
                pathWeaver.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(pathWeaver.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            powertrain::getPose,
            new RamseteController(pathWeaver.kRamseteB, pathWeaver.kRamseteZeta),
            new SimpleMotorFeedforward(
                pathWeaver.ksVolts,
                pathWeaver.kvVoltSecondsPerMeter,
                pathWeaver.kaVoltSecondsSquaredPerMeter),
            pathWeaver.kDriveKinematics,
            powertrain::getWheelSpeeds,
            new PIDController(pathWeaver.kPDriveVel, 0, 0),
            new PIDController(pathWeaver.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            powertrain::setVolts,
            powertrain);

    // Reset odometry to the starting pose of the trajectory.
    powertrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> powertrain.setVolts(0, 0));
  }
   
   /* if(autonomous.getSelected() == "trench"){ 
      
      return new SequentialCommandGroup(new Shoot(shooter, intake, powertrain, vision).withTimeout(3.6), 
                                      //  ramseteC(trajectoryPaths.get(0)),
                                        new PrintCommand("3"), 
                                        new PrintCommand("4")
                                        );
    }

    else if(autonomous.getSelected() == "mid"){
      
      return new SequentialCommandGroup(new PrintCommand("1"), new PrintCommand("2"), new PrintCommand("3"), new PrintCommand("4"));
    }
    
    else if(autonomous.getSelected() == "test"){
                                        return null;
      
    }
    else
    return null;
  } */
}
