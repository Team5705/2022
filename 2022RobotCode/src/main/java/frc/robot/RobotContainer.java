// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.kOI;
import frc.robot.Constants.pathWeaver;
import frc.robot.commands.ConveyorReverse;
import frc.robot.commands.Conveyor_input;
import frc.robot.commands.Drive;
import frc.robot.commands.GetBalls;
import frc.robot.commands.IntakeToggle;
import frc.robot.commands.ShootON;
import frc.robot.commands.SimpleTracking;
import frc.robot.commands.RoutinesCommands.AutoShooting;
import frc.robot.commands.ShooterCommands.AdjustHoodLoop;
import frc.robot.commands.ShooterCommands.AdjustShotLoop;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ControlEnergySystem;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Powertrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //Subsystems
  private final Powertrain powertrain;
  private final Shooter shooter;
  private final Conveyor conveyor;
  private final Intake intake;
  //public final Climber climber = new Climber();
  private final Hood hood;

  public final Vision vision;
  public final ControlEnergySystem controlEnergySystem;
  //public static Leds leds = new Leds();

  //Commands
  private final Drive drive;
  private final GetBalls getBalls;

  public static XboxController driverController;
  public static XboxController secondController;

  //Trajectory [] trajectories = new Trajectory[] {trajectory1};

  String trajectoryJSON1 = "paths/output/oi.wpilib.json";
  Trajectory trajectory1 = new Trajectory();

  String trajectoryJSON2 = "paths/output/io.wpilib.json";
  Trajectory trajectory2 = new Trajectory();

  String testTrajectory = "Test_path.wpilib.json";
  Trajectory trajectory3 = new Trajectory();

  SendableChooser<String> autonomous = new SendableChooser<String>();

  ArrayList<String> trajectoryPaths = new ArrayList<String>();

  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    powertrain = new Powertrain();
    shooter = new Shooter();
    conveyor = new Conveyor();
    intake = new Intake();
    //climber = new Climber();
    hood = new Hood();

    vision = new Vision();
    controlEnergySystem = new ControlEnergySystem();
    //leds = new Leds();

    //Commands
    drive = new Drive(powertrain);
    getBalls = new GetBalls(conveyor);

    driverController = new XboxController(kOI.controllerPort);
    secondController = new XboxController(kOI.controllerPort2);

    vision.ledsOn();
    readPaths();
    
    // Configure the button bindings
    configureButtonBindings();
    
    powertrain.setDefaultCommand(drive);
    conveyor.setDefaultCommand(getBalls);

    autonomous.addOption("mid", "mid");
    autonomous.addOption("onlyBack", "onlyBack");
    autonomous.addOption("oneball", "oneball");
    autonomous.addOption("null", "null");
    //autonomous.addOption("Emergency", "emergency");
    //autonomous.addOption("Test", "test");
    SmartDashboard.putData("autoMode", autonomous);
    
  }

  public void readPaths(){
    //tryReadPath(trajectoryJSON1, trajectory1);
    //tryReadPath(trajectoryJSON2, trajectory2);
    //tryReadPath(testTrajectory, trajectory3);
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(("pathplanner/generatedJSON/" + testTrajectory));
      trajectory3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      System.out.println("------------------------ pathplanner/generatedJSON/" + testTrajectory + " successfully read :D");
      
    } catch (IOException ex) {
      DriverStation.reportError("------------------------ " + "Unable to open path: " + testTrajectory, ex.getStackTrace());
      
    }
    /* try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
      trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      System.out.println(trajectoryJSON2 + " successfully read :D");
      
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open path: " + trajectoryJSON2, ex.getStackTrace());
      
    } */
  }

  /* public void tryReadPath(String name, Trajectory trajectory){
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON" + name);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      System.out.println(name + " successfully read :D");
      
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open path: " + name, ex.getStackTrace());
      System.out.println(name + " no read D:");
    }
  } */
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /* DRIVER 1 */
    //BUTTONS
    //new JoystickButton(driverController, 1).whileHeld(new SimpleTracking(powertrain, vision));
    new JoystickButton(driverController, 2).whileHeld(new AdjustShotLoop(shooter, vision));
    new JoystickButton(driverController, 3).whileHeld(new ShootON(shooter));
    //new JoystickButton(driverController, 4).whenPressed(new AutoShooting(powertrain, vision, shooter, conveyor));
    
    new JoystickButton(driverController, 5).whileHeld(new Conveyor_input(conveyor));
    new JoystickButton(driverController, 6).toggleWhenPressed(new IntakeToggle(intake));
    
    new JoystickButton(driverController, 7).whenPressed(new InstantCommand(powertrain::neutralModeBrake, powertrain)); //Chasis Brake mode
    new JoystickButton(driverController, 8).whenPressed(new InstantCommand(powertrain::neutralModeCoast, powertrain)); //Chasis Coast mode

    new JoystickButton(driverController, 9).whileHeld(new ConveyorReverse(conveyor, shooter));
    
    new JoystickButton(driverController, 10).whenPressed(new InstantCommand(powertrain::resetAll, powertrain)); //Reset  All

    /**POV**/
    //new POVButton(driverController, 90).whenPressed(new InstantCommand(vision::ledsOff, vision));
    //new POVButton(driverController, 270).whenPressed(new InstantCommand(vision::ledsOn, vision));
    
    //new POVButton(driverController, 0).whenPressed(new InstantCommand(climber::extend, climber));
    //new POVButton(driverController, 180).whenPressed(new InstantCommand(climber::contract, climber));
    
    
    /*DRIVER 2*/
    
    //new JoystickButton(driverController, 4).whenPressed(new AdjustHoodLoop(hood, 58.0));
    //new JoystickButton(driverController, 2).whenPressed(new AdjustHoodLoop(hood, 70.0));
    //new JoystickButton(secondController, 4).whileHeld(new RunCommand(() -> hood.moveHood(secondController.getRawAxis(1)), hood))
    //  .whenReleased(new InstantCommand(hood::neutralHood, hood));

  }
  
  public void updateAutonomous(){
      autonomous.getSelected();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return null;
    /* return new SequentialCommandGroup(
        new ParallelCommandGroup(
          runPath(trajectory1)//, 
          //new IntakeToggle(intake).withTimeout(5)
        ),
        new ParallelCommandGroup(
          //new ConveyorReverse(conveyor, shooter).withTimeout(1.5), 
          new SimpleTracking(powertrain, vision, true).withTimeout(1.2)
        ),
        new ParallelCommandGroup(
          new ShootON(shooter).withTimeout(2),
          new SequentialCommandGroup(
            new WaitCommand(1.5), new Conveyor_input(conveyor).withTimeout(0.5)
          )
        ),
        new SequentialCommandGroup(
          runPath(trajectory2)
        ),

       new ParallelCommandGroup(
          runPath(trajectory1)//, 
          //new IntakeToggle(intake).withTimeout(2)
        ),
        new AutoShooting(powertrain, vision, shooter, conveyor),
        runPath(trajectory2)
      ); */
      return runPath(trajectory3);
  }




  public Command runPath(Trajectory myTrajectory){
    RamseteCommand ramseteCommand =
    new RamseteCommand(
        myTrajectory,
        powertrain::getPose,
        new RamseteController(),
        new SimpleMotorFeedforward(
          pathWeaver.ksVolts,
          pathWeaver.kvVoltSecondsPerMeter,
          pathWeaver.kaVoltSecondsSquaredPerMeter),
          pathWeaver.kDriveKinematics,
          powertrain::getWheelSpeeds,
          new PIDController(0, 0, 0),
          new PIDController(0, 0, 0),
          // RamseteCommand passes volts to the callback
          powertrain::setVolts,
      powertrain);

    // Reset odometry to the starting pose of the trajectory.
    /*powertrain.resetOdometry(myTrajectory.getInitialPose());*/
    
    // Run path following command, then stop at the end.
    return new SequentialCommandGroup(//new InstantCommand(() -> powertrain.resetOdometry(myTrajectory.getInitialPose()), powertrain), 
                                      ramseteCommand.andThen(() -> powertrain.setVolts(0, 0))//,
                                      //new InstantCommand(() -> powertrain.resetOdometry(myTrajectory.getInitialPose()), powertrain)
                                      );
    //return ramseteCommand;
  }
}