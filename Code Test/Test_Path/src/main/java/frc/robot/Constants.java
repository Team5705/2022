// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstant {
        public static final int[] portsMotors = new int[] { 4,    // leftMaster(SRX) 
                                                            5,   // leftFollow(SPX)
                                                            3,    // rightMaster(SRX)  
                                                            9 }; // rightFollow(SPX)

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
        public static final double xInitialPosition = 0; // x
        public static final double yInitialPosition = 0; // y
        public static final double initialHeading = 0; //degrees

        public static final double ksVolts = 0.66905;
        public static final double kvVoltSecondsPerMeter = 2.9555;
        public static final double kaVoltSecondsSquaredPerMeter = 2.15361;

        public static final double kPDriveVel = 2.5029;

        public static final double kTrackwidthMeters = Units.inchesToMeters(23.25); // Distancia horizontal entre las ruedas en metros
                                                                                 
        public static final DifferentialDriveKinematics kDriveKinematics = 
            new DifferentialDriveKinematics(kTrackwidthMeters);

        //Only for custom path, wpi.json not
        public static final double kMaxSpeedMetersPerSecond = 3; // Velocidad maxima del robot en metros por segundo
        public static final double kMaxAccelerationMetersPerSecondSquared = 3; // Aceleracion maxima del robot en metros por segundo

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        // Debería de funcionar bien en cualquier robot, de ser necesario la
        // modificacion entrar a la documentacion de WPI
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
