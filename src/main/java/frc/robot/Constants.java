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
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
        public static final double TRACKWIDTH_METERS = Units.inchesToMeters(29.0); // FIXME Measure and set trackwidth
        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * Should be measured from center to center.
         */
        public static final double WHEELBASE_METERS = Units.inchesToMeters(29.0); // FIXME Measure and set wheelbase

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(DRIVE.TRACKWIDTH_METERS / 2.0, DRIVE.WHEELBASE_METERS / 2.0),
                new Translation2d(DRIVE.TRACKWIDTH_METERS / 2.0, -DRIVE.WHEELBASE_METERS / 2.0),
                new Translation2d(-DRIVE.TRACKWIDTH_METERS / 2.0, DRIVE.WHEELBASE_METERS / 2.0),
                new Translation2d(-DRIVE.TRACKWIDTH_METERS / 2.0, -DRIVE.WHEELBASE_METERS / 2.0)
        );

        public static final double WHEEL_DIAMETER_METERS = 0.10033; // .10033 = ~4 inches
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

        public static final int PIGEON_ID = 5; // FIXME Set Pigeon ID

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 11; // FIXME Set front left module drive motor ID
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 12; // FIXME Set front left module steer motor ID
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 13; // FIXME Set front left steer encoder ID
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(28.531); // 189.492FIXME Measure and set front left steer offset

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 21; // FIXME Set front right drive motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 22; // FIXME Set front right steer motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 23; // FIXME Set front right steer encoder ID
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(174.638); // 137FIXME Measure and set front right steer offset

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 41; // FIXME Set back left drive motor ID
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 42; // FIXME Set back left steer motor ID
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 43; // FIXME Set back left steer encoder ID
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(99.492); // 326.777177.803FIXME Measure and set back left steer offset

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 31; // FIXME Set back right drive motor ID
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 32; // FIXME Set back right steer motor ID
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 33; // FIXME Set back right steer encoder ID
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(238); // 300FIXME Measure and set back right steer offset        

        // Drivetrain Performance Mechanical limits
        static public final double MAX_FWD_REV_SPEED_MPS = Units.feetToMeters(12.0);
        static public final double MAX_STRAFE_SPEED_MPS = Units.feetToMeters(12.0);
        static public final double MAX_ROTATE_SPEED_RAD_PER_SEC = Units.degreesToRadians(180.0);
        static public final double MAX_TRANSLATE_ACCEL_MPS2 = MAX_FWD_REV_SPEED_MPS/0.25; //0-full time of 0.25 second
        static public final double MAX_ROTATE_ACCEL_RAD_PER_SEC_2 = MAX_ROTATE_SPEED_RAD_PER_SEC/0.25; //0-full time of 0.25 second

        // Fine control speed limits
        static public final double MAX_FWD_REV_SPEED_MPS_SLOW = Units.feetToMeters(6.0);
        static public final double MAX_STRAFE_SPEED_MPS_SLOW = Units.feetToMeters(6.0);
        static public final double MAX_ROTATE_SPEED_RAD_PER_SEC_SLOW = Units.degreesToRadians(90.0);

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // SENSOR CONSTANTS
        // Sensor-related constants - pulled from datasheets for the sensors and gearboxes
        static public final int ENC_PULSE_PER_REV = 2048; // TalonFX integrated sensor
        static public final int WHEEL_ENC_COUNTS_PER_WHEEL_REV = ENC_PULSE_PER_REV;  //Assume 1-1 gearing for now
        static public final int STEER_ENC_COUNTS_PER_MODULE_REV = 4096; // CANCoder
        static public final double WHEEL_ENC_WHEEL_REVS_PER_COUNT  = 1.0/((double)(WHEEL_ENC_COUNTS_PER_WHEEL_REV));
        static public final double steer_ENC_MODULE_REVS_PER_COUNT = 1.0/((double)(STEER_ENC_COUNTS_PER_MODULE_REV));

        public static final double AIMkP = 0.001;
        public static final double AIMFF = 0.08;
    }

    public static final class AUTO {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double TRAJECTORYXkP = 10;
        public static final double TRAJECTORYYkP = 1;
        public static final double THETACONTROLLERkP = 1;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints THETACONTROLLERCONSTRAINTS =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final double AIMROTATE = 0.3;
    }

    public static final class ROBOT {
        public static final double MASS_kg = Units.lbsToKilograms(140);
        public static final double MOI_KGM2 = 1.0/12.0 * ROBOT.MASS_kg * Math.pow((DRIVE.TRACKWIDTH_METERS*1.1),2) * 2; //Model moment of intertia as a square slab slightly bigger than wheelbase with axis through center

        public static final double QUIESCENT_CURRENT_DRAW_A = 2.0; //Misc electronics
        public static final double BATTERY_NOMINAL_VOLTAGE = 13.2; //Nicely charged battery
        public static final double BATTERY_NOMINAL_RESISTANCE = 0.040; //40mOhm - average battery + cabling
        public static final double MAX_VOLTAGE = 12.0; // Maximum Voltage sent to a motor controller

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

    public static final class LED {
        public static final int PWMPORT = 0;
        public static final int BUFFERSIZE = 120;
    }
    
    public static final class OI {
        public static final int DRVCONTROLLERPORT = 0;
        public static final int OPCONTROLLERPORT = 1;
        public static final boolean PRACTICE = true;
    }

    public static final class INTAKE {
        public static final int CANID = 5;
        public static final int CANID2 = 6;
        
        public static final int SOLENOID1FWD = 3;
        public static final int SOLENOID1REV = 2;

        public static final double SPEED = 0.75;

        public static final double STALLCURRENT = 20;
        public static final int STALLTIME = 500;
        public static final int PDPSLOT = 7;
    }

    public static final class CONVEYOR {
        public static final int CANID = 10;

        public static final int FRONTSENSORPORT = 1;
        public static final int TOPSENSORPORT = 0;

        public static final double SPEED = 0.2;
        public static final double SHOOTSPEED = 0.4;
        public static final double BACKSPEED = -0.2;
    }

    public static final class CLIMB {
        public static final int CANID = 8;

        public static final double P = 0.01;
        public static final double TOLERANCE = 0.01;
    }

    public static final class CAMERA {
        public static final double BALLCAMERAANGLE = 0; // Degrees
        public static final double SHOOTERCAMERAANGLE = 0; // Degrees
        public static final double BALLCAMERAHEIGHT = .8; // Meters
        public static final double SHOOTERCAMERAHEIGHT = .8; // Meters
        public static final double BALLTARGETHEIGHT = Units.inchesToMeters(9.5); // Height to top of the ball
        // public static final double kMinimumRange = 4; // Meters
        // public static final double kMaximumRange = -9; // Meters
        public static final int LIMELIGHTPIPELINE = 0;
        public static final int HD3000PIPELINE = 0;
    }

    public static final class SHOOTER {
        public static final int MOTORPORT = 14;
        public static final int MOTOR2PORT = 15;
        public static final int HOODPORT = 16;
    
        public static final double HOODkP = 0.01;
        public static final int HOODCIRCLE = -58500;
        public static final int HOODLOW = -30000;

        public static final double TOLERANCERPS = 6.0;
        
        public static final double WHEELDIAMETERINCHES = 4;
        public static final int ENCODERCPR = 2048;
        // Multiply by 10 to get Raw per second.  Divide by encoder CPR to get rotations
        public static final double RAWTOFLYWHEELRPS = 10 / (double) ENCODERCPR;

        // Multiply by encoder CPR to get raw counts per second.  Divide by 10 to get per decisecond
        public static final double RPSTORAW = (double) ENCODERCPR / 10;

        public static final double LOWFF = 0.24;
        public static final double FENDERFF = 0.45; //.38 for low energy shot
        public static final double CIRCLEFF = 0.52;
        public static final double P = 0;//999999999999999.0;
        public static final double D = 0;
    
        // On a real robot the feedforward constants should be empirically determined; these are
        // reasonable guesses.
        public static final double kSVOLTS = 0.73856;
        public static final double kVVOLTSECONDSPERROTATION = 0.11106;
        public static final double kA = 0.0028227;

        public static final double SPEEDCHANGE = 0.01;
        public static final double SETPOINT2 = 2.0;
        public static final double SETPOINT4 = 4.0;
    }
}
