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
        public static final int[] portsMotors = new int[] { 3,    // leftMaster(SRX) 
                                                            4,   // leftFollow(SPX)
                                                            5,    // rightMaster(SRX)  
                                                            6 }; // rightFollow(SPX)

        public static final Port Gyro = Port.kOnboardCS0;

    }
    
    public static final class OIConstant {
        public static final int controllerPort = 0;
    }
    
    public static final class IntakeConstant {
        public static final int m1 = 10; // Intake
        
        public static final int[] solenoids = new int[] { 3 };
        public static final int[] sensors = new int[] { 0,   //
            3,   // B  Banda B
            1,   // C  Banda B
            2 }; // D  Banda B
    }

    public static final class ConveyorConstant {
        public static final int m1 = 40;
    }
        
        
    public static final class Shoot {
        public static final int mShooter = 38;  //Motor A | Izquierdo
        public static final int mShooter2 = 39; //Motor B | Derecho
    }
        
    public static final class pathWeaver {
        public static final double xInitialPosition = 0; // x IMPORTANT | Meters
        public static final double yInitialPosition = 0; // y IMPORTANT | Meters
        public static final double initialHeading = 0; //degrees

        public static final double ksVolts = 0.93394;
        public static final double kvVoltSecondsPerMeter = 2.992;
        public static final double kaVoltSecondsSquaredPerMeter = 0.8227;

        public static final double kPDriveVel = 3.8797;

        public static final double kTrackwidthMeters = Units.inchesToMeters(23.75); // Distancia horizontal entre las ruedas en metros
                                                                                 
        public static final DifferentialDriveKinematics kDriveKinematics = 
            new DifferentialDriveKinematics(kTrackwidthMeters);

        //Only for custom path, wpi.json not
        public static final double kMaxSpeedMetersPerSecond = 2; // Velocidad maxima del robot en metros por segundo
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.5; // Aceleracion maxima del robot en metros por segundo

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        // Debería de funcionar bien en cualquier robot, de ser necesario la
        // modificacion entrar a la documentacion de WPI
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
