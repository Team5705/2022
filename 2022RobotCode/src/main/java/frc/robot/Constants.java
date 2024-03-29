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
    public static final class kGlobal {
        public static final int portPDP = 1;
        public static final int portPCM = 30;
    }
    public static final class kDrive {
        public static final int[] portsMotors = new int[] { 38,   //leftMaster
                                                            3,   //leftFollow
                                                            4,   //rightMaster
                                                            41 }; //rightFollow

        public static final int[] channelsMotors = new int[] { 15,
                                                               13,
                                                               1, 
                                                               3 };                      

        public static final Port Gyro = Port.kOnboardCS0;

        public static final int leftEncoder = 50;
        public static final int rightEncoder = 51;

    }
    
    public static final class kOI {
        public static final int controllerPort = 0;
        public static final int controllerPort2 = 1;
    }
    
    public static final class kIntake {
        //public static final int m1 = 10; // Intake
        public static final int m1 = 60; // Intake
        public static final int channel = 12;
        
        public static final int[] solenoids = {2};
        public static final int[] sensors = new int[] { 0 };

        public static final double speed = 0.7;
    }

    public static final class kConveyor {
        public static final int mSUPP = 5;
        public static final int mADC = 6;
        //public static final int m1 = 35;
        //public static final int m2 = 36;

        public static final int mainSensor = 9;

        public static final int channel_mSUPP = 11;
        public static final int channel_mADC = 2;

        public static final double kSpeedGlobal = 0.2;
    }
        
        
    public static final class kShooter {
        public static final int mShooterA = 24;  //Motor A | Izquierdo
        public static final int mShooterB = 39; //Motor B | Derecho

        public static final int channel_m1 = 14;
        public static final int channel_m2 = 12;

        public static final int servoLeft = 8;
        public static final int servoRight = 9;
        
        public static final int channel_hood = 2; //Power Servo Module
    }

    public static final class kClimber {
        public static final int leftSolenoid = 4;
        public static final int rightSolenoid = 5;
    }
        
    public static final class pathWeaver {
        public static final double xInitialPosition = 5; // x IMPORTANT | Meters
        public static final double yInitialPosition = 3; // y IMPORTANT | Meters
        public static final double initialDegree = 180; //degrees

        public static final double ksVolts = 0.12903;
        public static final double kvVoltSecondsPerMeter = 2.8981;
        public static final double kaVoltSecondsSquaredPerMeter = 0.50473;

        public static final double kPDriveVel = 3.8672;

        public static final double kTrackwidthMeters = Units.inchesToMeters(23); // Distancia horizontal entre las ruedas en metros
                                                                                 
        public static final DifferentialDriveKinematics kDriveKinematics = 
            new DifferentialDriveKinematics(kTrackwidthMeters);

        //*** Only for custom path, wpi.json not ***//
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
