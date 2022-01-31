/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.numbers.N2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DRIVE {
        /**
         * The left-to-right distance between the drivetrain wheels
         *
         * Should be measured from center to center.
         */
        public static final double TRACKWIDTH_METERS = 1.0; // FIXME Measure and set trackwidth
        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * Should be measured from center to center.
         */
        public static final double WHEELBASE_METERS = 1.0; // FIXME Measure and set wheelbase

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(DRIVE.TRACKWIDTH_METERS / 2.0, DRIVE.WHEELBASE_METERS / 2.0),
                new Translation2d(DRIVE.TRACKWIDTH_METERS / 2.0, -DRIVE.WHEELBASE_METERS / 2.0),
                new Translation2d(-DRIVE.TRACKWIDTH_METERS / 2.0, DRIVE.WHEELBASE_METERS / 2.0),
                new Translation2d(-DRIVE.TRACKWIDTH_METERS / 2.0, -DRIVE.WHEELBASE_METERS / 2.0)
        );

        public static final double WHEEL_DIAMETER_METERS = 0.10033; // .10033 = ~4 inches
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

        public static final int PIGEON_ID = 0; // FIXME Set Pigeon ID

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 0; // FIXME Set front left module drive motor ID
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1; // FIXME Set front left module steer motor ID
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 0; // FIXME Set front left steer encoder ID
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set front left steer offset

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2; // FIXME Set front right drive motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 3; // FIXME Set front right steer motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 1; // FIXME Set front right steer encoder ID
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set front right steer offset

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 4; // FIXME Set back left drive motor ID
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 5; // FIXME Set back left steer motor ID
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 2; // FIXME Set back left steer encoder ID
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set back left steer offset

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6; // FIXME Set back right drive motor ID
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7; // FIXME Set back right steer motor ID
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3; // FIXME Set back right steer encoder ID
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set back right steer offset        

        // Drivetrain Performance Mechanical limits
        static public final double MAX_FWD_REV_SPEED_MPS = Units.feetToMeters(12.0);
        static public final double MAX_STRAFE_SPEED_MPS = Units.feetToMeters(12.0);
        static public final double MAX_ROTATE_SPEED_RAD_PER_SEC = Units.degreesToRadians(360.0);
        static public final double MAX_TRANSLATE_ACCEL_MPS2 = MAX_FWD_REV_SPEED_MPS/0.25; //0-full time of 0.25 second
        static public final double MAX_ROTATE_ACCEL_RAD_PER_SEC_2 = MAX_ROTATE_SPEED_RAD_PER_SEC/0.25; //0-full time of 0.25 second
        public static final double MAX_VOLTAGE = 12.0; // Maximum Voltage sent to the drive motors

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // SENSOR CONSTANTS
        // Sensor-related constants - pulled from datasheets for the sensors and gearboxes
        static public final int ENC_PULSE_PER_REV = 2048; // TalonFX integrated sensor
        static public final int WHEEL_ENC_COUNTS_PER_WHEEL_REV = ENC_PULSE_PER_REV;  //Assume 1-1 gearing for now
        static public final int STEER_ENC_COUNTS_PER_MODULE_REV = 4096; // CANCoder
        static public final double WHEEL_ENC_WHEEL_REVS_PER_COUNT  = 1.0/((double)(WHEEL_ENC_COUNTS_PER_WHEEL_REV));
        static public final double steer_ENC_MODULE_REVS_PER_COUNT = 1.0/((double)(STEER_ENC_COUNTS_PER_MODULE_REV));
    }

    public static final class AUTO {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double TRAJECTORYXkP = 1;
        public static final double TRAJECTORYYkP = 1;
        public static final double THETACONTROLLERkP = 1;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints THETACONTROLLERCONSTRAINTS =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class ROBOT {
        public static final double MASS_kg = Units.lbsToKilograms(140);
        public static final double MOI_KGM2 = 1.0/12.0 * ROBOT.MASS_kg * Math.pow((DRIVE.TRACKWIDTH_METERS*1.1),2) * 2; //Model moment of intertia as a square slab slightly bigger than wheelbase with axis through center

        public static final double QUIESCENT_CURRENT_DRAW_A = 2.0; //Misc electronics
        public static final double BATTERY_NOMINAL_VOLTAGE = 13.2; //Nicely charged battery
        public static final double BATTERY_NOMINAL_RESISTANCE = 0.040; //40mOhm - average battery + cabling

        // Assumed starting location of the robot. Auto routines will pick their own location and update this.
        static public final Pose2d DFLT_START_POSE = new Pose2d(Units.feetToMeters(24.0), Units.feetToMeters(10.0), Rotation2d.fromDegrees(0));

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // ROBOT PHYSICAL CONSTANTS
        // Robot physical dimensions and mass quantities.
        static public final double ROBOT_MASS_kg = Units.lbsToKilograms(140);
        static public final double ROBOT_MOI_KGM2 = 1.0/12.0 * ROBOT_MASS_kg * Math.pow((DRIVE.WHEELBASE_METERS*1.1),2) * 2; //Model moment of intertia as a square slab slightly bigger than wheelbase with axis through center
        // Location of vision camera relative to robot center - currently front middle.
        static public final Transform2d robotToCameraTrans = new Transform2d(new Translation2d(DRIVE.WHEELBASE_METERS/2, 0), new Rotation2d(0.0));

        // Vision Camera
        static public final String PHOTON_CAM_NAME = "MainCamera";
    }

    public static final class DriveConstants {
        // The CAN Id that each of these devices is configured with
        public static final int kLeftMotor1Port = 1;
        public static final int kLeftMotor2Port = 2;
        public static final int kRightMotor1Port = 3;
        public static final int kRightMotor2Port = 4;
        public static final int kPigeonPort = 0;
    
        // Differential Drive setup parameters.  These control how the drivers input translates to motor power.
        public static final double kMaxOutputForward = 1; // % motor ouput
        public static final double kMaxOutputForwardSlow = 0.3; // % motor ouput
        public static final double kMaxOutputRotation = 0.5; // % motor ouput
        public static final double kMaxOutputRotationSlow = 0.25; // % motor ouput
        public static final double kDeadbandForward = 0.05; // % motor ouput
        public static final double kDeadbandRotation = 0.05; // % motor ouput
        public static final double kRampForward = 0.2; // Seconds to go from min to max motor %
        public static final double kRampRotation = 0.2; // Seconds to go from min to max motor %
        public static final double kMinOutputForward = 0; // Minimum % forward power
        public static final double kMinOutputRotation = 0.2; // Minimum % rotation power
        public static final double kDriveStraightLeft = 1; // Multiplier on motor power to help drive straight
        public static final double kDriveStraightRight = 0.98; // Multiplier on motor power to help drive straight

        public static final DCMotor kDriveGearbox = DCMotor.getCIM(2);
        public static final double kDriveGearing = 10.71;

        // Baseline values for a RAMSETE follower in units of meters and seconds
        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;

        /** Voltage needed to overcome the motor’s static friction. kS */
        public static final double kS = 1.24; //0.112 // 1.01 pneumatic //.829

        /** Voltage needed to hold (or "cruise") at a given constant velocity. kV */
        public static final double kV = 2.92; //0.222; //2.93 pneumatic //3.04

        /** Voltage needed to induce a given acceleration in the motor shaft. kA */
        public static final double kA = 0.474; //0.00128; //0.761 pneumatic //.676

        // Simulation Drivetrain stuff
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        // These two values are "angular" kV and kA
        public static final double kvVoltSecondsPerRadian = 1.5;
        public static final double kaVoltSecondsSquaredPerRadian = 0.3;

        public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
            LinearSystemId.identifyDrivetrainSystem(kV, kA, kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);
        
        /* LinearSystemId.identifyDrivetrainSystem(kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter,
                kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian); */
        
        public static final int kEncoderCPR = 4096;
        public static final double kWheelDiameterInches = 8d;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(kWheelDiameterInches);
        public static final double kWheelCircumferenceInches = kWheelDiameterInches * Math.PI;
        public static final double kWheelCircumferenceMeters = Units.inchesToMeters(kWheelDiameterInches) * Math.PI;

        public static final double kTrackWidthMeters = 1.00639; // 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackWidthMeters);

        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            kWheelCircumferenceMeters / (double) kEncoderCPR; // .00015585

        /**
         * This is a property of the Pigeon IMU, and should not be changed.
         */
        public final static int kPigeonUnitsPerRotation = 8192;

        /**
         * Using the configSelectedFeedbackCoefficient() function, scale units to 3600 per rotation.
         * This is nice as it keeps 0.1 degrees of resolution, and is fairly intuitive.
         */
        public final static double kTurnTravelUnitsPerRotation = 3600;

        /**
         * Set to zero to skip waiting for confirmation.
         * Set to nonzero to wait and report to DS if action fails.
         */
        public final static int kTimeoutMs = 30;

        /**
         * PID Gains may have to be adjusted based on the responsiveness of control loop.
         * kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity units at 100% output
         * Not all set of Gains are used in this project and may be removed as desired.
         * 
         * 	                                    			  kP   kI   kD   kF               Iz    PeakOut */
        public final static Gains kGains_Distanc = new Gains( 0.1, 0.0,  0.0, 0.2,            100,  0.30 );
        public final static Gains kGains_Turning = new Gains( 0.05, 0.0,  4.0, 100,            200,  0.3 );
        public final static Gains kGains_Velocit = new Gains( 1.2, 0.0, 0, 1023.0/6800.0,  300,  1.00 ); //0.2 // .00712
        public final static Gains kGains_MotProf = new Gains( 1.0, 0.0,  0.0, 1023.0/6800.0,  400,  1.00 );
        
        /** ---- Flat constants, you should not need to change these ---- */
        /* We allow either a 0 or 1 when selecting an ordinal for remote devices [You can have up to 2 devices assigned remotely to a talon/victor] */
        public final static int REMOTE_0 = 0;
        public final static int REMOTE_1 = 1;
        /* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
        public final static int PID_PRIMARY = 0;
        public final static int PID_TURN = 1;
        /* Firmware currently supports slots [0, 3] and can be used for either PID Set */
        public final static int SLOT_0 = 0;
        public final static int SLOT_1 = 1;
        public final static int SLOT_2 = 2;
        public final static int SLOT_3 = 3;
        /* ---- Named slots, used to clarify code ---- */
        public final static int kSlot_Distanc = SLOT_0;
        public final static int kSlot_Turning = SLOT_1;
        public final static int kSlot_Velocit = SLOT_2;
        public final static int kSlot_MotProf = SLOT_3;

        // Everything below is for either Absolute or Relative Turns
        public static final double kMaxTurnRateDegPerS = 100;
        public static final double kMaxTurnAccelerationDegPerSSquared = 300;

        public static final double kTurnToleranceDeg = 5;
        public static final double kTurnRateToleranceDegPerS = 10; // degrees per second

        public static final double kRelTurnP = .1;
        public static final double kRelTurnI = 0;
        public static final double kRelTurnD = 0.15;
        public static final double kRelTurnFriction = 0.5;

        public static final double kMaxRelTurnRateDegPerS = 20;
        public static final double kMaxRelTurnAccelerationDegPerSSquared = 300;

        public static final double kRelTurnToleranceDeg = 1;
        public static final double kRelTurnRateToleranceDegPerS = 3; // degrees per second

        // Turn angle constraints
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;

        // Turn angle PID
        public static final double kTurnP = .1;
        public static final double kTurnI = 0;
        public static final double kTurnD = 0.15;

        // Drive Straight PID
        public static final double kStraightP = .1;
        public static final double kStraightI = 0;
        public static final double kStraightD = 0;

        // Turn Velocity constraints
        public static final double kVelocityMaxSpeedMetersPerSecond = 3;
        public static final double kVelocityMaxAccelerationMetersPerSecondSquared = 2;

        // Turn Velocity PID
        public static final double kVelocityP = .1;
        public static final double kVelocityI = 0;
        public static final double kVelocityD = 0.15;
    }
    
    public static final class ShooterConstants {
        public static final int[] kEncoderPorts = new int[]{0, 1};
        public static final boolean kEncoderReversed = true;
        public static final double kGearRatio = 2.0;
        // 18730 (775pro RPM) / 600 = 31.21666
        // 4096 (sensor units per rotation) / 4 = 1024 * 31.21666 = 31965.866
        public static final int kShooterMotorPort = 10;
        public static final int kShooterMotorPort2 = 11;
    
        public static final double kShooterFarTrenchRPS = 112;
        public static final double kShooter1 = 116;
        public static final double kShooter2 = 114;
        public static final double kShooter3 = 112;
        public static final double kShooter4 = 110;
        public static final double kShooterToleranceRPS = 6.0;
        public static final double kShooterToleranceAccel = 30.0;
        
        public static final double kSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(500.0);

        public static final int kEncoderCPR = 2048;
        public static final double kWheelDiameterInches = 4;
        public static final double kEncoderDistancePerPulse =
            // Distance units will be rotations per second
            1.0 / (double) kEncoderCPR;

        // These are not real PID gains, and will have to be tuned for your specific robot.
        public static final double kP = 0.12; // .000321 / 1.2 was competition number
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0; //Not used
    
        // On a real robot the feedforward constants should be empirically determined; these are
        // reasonable guesses.
        public static final double kSVolts = 1;//0.87; // .05 original / .92 comp
        public static final double kVVoltSecondsPerRotation = 0.0774; // 12 / .0764 comp
        public static final double kA = 0.00003; //.0429 comp
    }
    
    public static final class ControlPanelConstants {
        public static final int kSolenoidPort = 0;
        public static final int kSpinWheelPort = 33;
        public static final double kWheelSpeedFast = 0.6;
        public static final double kWheelSpeedSlow = 0.1;
        public static final double colorwheel_past = 2;
        public static final double colorwheel_slow = 0.2;
        public static final double colorwheel_fast = 0.3;
        public static final double colorwheel_ticks = 110;
    }

    public static final class AutoConstants {
        public static final double kAutoTimeoutSeconds = 12;
        public static final double kAutoShootInit = 3;
        public static final double kAutoShootRest = 6;

        public static final double kAutoShootRPS = 112;
        
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // %of Picutre the Ball should fill if Red 0-100% = 0-1
        public static final double kBallArea = 0.5;
    }

    public static final class LEDConstants {
        public static final int kLEDPWMPort = 0;
        public static final int kBufferSize = 60;
    }
    
    public static final class OI {
        public static final int DRVCONTROLLERPORT = 0;
        public static final int OPCONTROLLERPORT = 1;
        public static final boolean PRACTICE = true;
    }

    public static final class IntakeConstants {
        public static final int kIntakeControllerPort = 5;
        
        public static final int kSolenoid1ControllerPort = 0;
        public static final int kSolenoid2ControllerPort = 1;
        public static final int kSolenoid3ControllerPort = 4;
        public static final int kSolenoid4ControllerPort = 5;

        public static final double kIntakeMotorSpeed = 0.5;

        public static final double kIntakeStallCurrent = 20;
        public static final int kStallTimeMS = 500;
        public static final int kIntakePDPSlot = 7;
    }

    public static final class ConveyorConstants {
        public static final int kConveyor1ControllerPort = 6;
        public static final int kConveyor2ControllerPort = 7;

        public static final double kConveyorTopMotorSpeed = 1.0;
        public static final double kConveyorBottomMotorSpeed = 0.2;
        public static final double kConveyorBackSpeed = -0.5;
    }

    public static final class ClimbConstants {
        public static final int kClimbLeftControllerPort = 11;
        public static final int kClimbRightControllerPort = 10;
        public static final int kEncoderCPR = 4096;
        public static final double kClimbP = 0.5;
        public static final double kErrorTolerance = 800;

        public static final int kFullUpEncoderCount = 42000;
        public static final int kOnBarEncoderCount = 53000;
        public static final int kHangingEncoderCount = 63000;
    }

    public static final class CAMERA {
        public static final double BALLCAMERAANGLE = 0; // Degrees
        public static final double SHOOTERCAMERAANGLE = 0; // Degrees
        public static final double BALLCAMERAHEIGHT = .8; // Meters
        public static final double SHOOTERCAMERAHEIGHT = .8; // Meters
        public static final double BALLTARGEHEIGHT = Units.inchesToMeters(9.5); // Height to top of the ball
        public static final double SHOOTERTARGEHEIGHT = Units.inchesToMeters(81.19); // Height to bottom of the tape
        // public static final double kMinimumRange = 4; // Meters
        // public static final double kMaximumRange = -9; // Meters
        public static final int LIMELIGHTPIPELINE = 0;
        public static final int HD3000PIPELINE = 0;
    }

    public static final class SHOOTER {
        public static final boolean kEncoderReversed = true;
        public static final double kGearRatio = 2.0;
        // 18730 (775pro RPM) / 600 = 31.21666
        // 4096 (sensor units per rotation) / 4 = 1024 * 31.21666 = 31965.866
        public static final int kShooterMotorPort = 10;
        public static final int kShooterMotorPort2 = 11;
    
        public static final double kShooterFarTrenchRPS = 112;
        public static final double kShooter1 = 116;
        public static final double kShooter2 = 114;
        public static final double kShooter3 = 112;
        public static final double kShooter4 = 110;
        public static final double TOLERANCERPS = 6.0;
        public static final double kShooterToleranceAccel = 30.0;
        
        public static final double kSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(500.0);

        public static final int kEncoderCPR = 2048;
        public static final double kWheelDiameterInches = 4;
        public static final double kEncoderDistancePerPulse =
            // Distance units will be rotations per second
            1.0 / (double) kEncoderCPR;

        public static final double P = 0.12;
        public static final double D = 0;
    
        // On a real robot the feedforward constants should be empirically determined; these are
        // reasonable guesses.
        public static final double kSVolts = 1;//0.87; // .05 original / .92 comp
        public static final double kVVoltSecondsPerRotation = 0.0774; // 12 / .0764 comp
        public static final double kA = 0.00003; //.0429 comp
    }
}
