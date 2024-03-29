/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstant {
        public static final int[] portsMotors = new int[] { 4,    // leftMaster(SRX) 
                                                            32,   // leftFollow(SPX)
                                                            3,    // rightMaster(SRX)  
                                                            43 }; // rightFollow(SRX)

        public static final Port Gyro = Port.kOnboardCS0;
    }

    public static final class OIConstant {
        public static final int controllerPort = 0;
    }

    public static final class Intake {
        public static final int m1 = 2; // Intake

        public static final int m2 = 0; // Banda A
        public static final int m3 = 1; // Banda B

        public static final int[] solenoids = new int[] { 3 };
        public static final int[] sensors = new int[] { 0,   // A  Buffer --> Banda A
                                                        1,   // B  Banda B
                                                        2,   // C  Banda B
                                                        3 }; // D  Banda B
    }


    public static final class Shoot {
        public static final int mShooter = 6;  //Motor A
        public static final int mShooter2 = 8; //Motor B
    }

    public static final class pathWeaver {
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining
        // these
        // values for your robot.
        public static final double ksVolts = 1.01;
        public static final double kvVoltSecondsPerMeter = 3.01;
        public static final double kaVoltSecondsSquaredPerMeter = 0.632;

        public static final double WHEEL_RADIUS = .0762;
        public static final double TICKS_TO_METERS_RATIO = (2 * Math.PI * WHEEL_RADIUS) / 4096;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 0.003;

        public static final double kTrackwidthMeters = Units.inchesToMeters(23.25); // Distancia horizontal entre las
                                                                                    // ruedas en metros
                                                                                 
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 1.2; // Velocidad maxima del robot en metros por segundo
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.2; // Aceleracion maxima del robot en
                                                                                 // metros por segundo

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        // Debería de funcionar bien en cualquier robot, de ser necesario la
        // modificacion entrar a la documentacion de WPI
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.7;

        public static final boolean kGyroReversed = true;
    }
}
