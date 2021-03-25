package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

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

    /*
     * Follow these variable naming conventions: - Motor names should be named
     * prefixed by the subsystem they are a part of, followed by its function and
     * the number of the motor - Buttons should be prefixed by the controller they
     * are on, followed by the command it is bound to - Sensor names should be
     * prefixed by subsystem, followed by type or function and number
     */

    // Drive Characterization Values
    //public static final double ksVolts = 0.143;
    //public static final double kvVoltsSecondsPerMeter = 2.57;
    //public static final double kaVoltsSecondsSquaredPerMeter = 0.389;
    //public static final double kPDriveVel = 0.581;
    //public static final double kDDriveVel = 270.0;
    //public static final double kTrackWidthMeters = 1.772; this is so wrong lmao

    public static final double ksVolts = 0.167; // Value obtained from characterization
    public static final double kvVoltsSecondsPerMeter = 2.63; // Value obtained from characterization
    public static final double kaVoltsSecondsSquaredPerMeter = 0.425; // Value obtained from characterization
    public static final double kPDriveVel = 0.00289; // Value obtained from characterization
    public static final double kDDriveVel = 0.0; // Value obtained from characterization
    public static final double kTrackWidthMeters = Units.inchesToMeters(23.0); // value obtained from physically measuring wheel
    public static final double kMaxSpeedMetersPerSecond = 1.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.0;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // (1, 1) on field is (0.762, 0.762) in meters
    public static final double fieldUnitsToMeters = 2.5 / 3.28; // Conversion from 1 meter to 2.5 ft
    public static final double gearRatio = 12.75; // 12.75 motor rotations : 1 wheel rotation
    public static final double wheelDiameterInMeters = Units.inchesToMeters(7.75); // Wheel diameter in meters
    public static final double centerRobotToIntakeMeters = Units.inchesToMeters(15.5);

    // Mecanum Kinematics Values
    //public static final MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(Constants.m_frontLeft, Constants.m_frontRight, Constants.m_backLeft, Constants.m_backRight);
    public static final DifferentialDriveKinematics kDifferentialKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
    private static final double translationXMeters = 0.2604; // Distance of wheel from center of robot moving to the front
    private static final double translationYMeters = 0.2668; // Distance of wheel from center of robot moving to the left
    public static final Translation2d m_frontLeft = new Translation2d(translationXMeters, translationYMeters);
    public static final Translation2d m_frontRight = new Translation2d(translationXMeters, -translationYMeters);
    public static final Translation2d m_backLeft = new Translation2d(-translationXMeters, translationYMeters);
    public static final Translation2d m_backRight = new Translation2d(-translationXMeters, -translationYMeters);
    
    /* Motors */
    // Intake
    public static final int INTAKE_PRIMARY = 5;
    public static final int INTAKE_CONVEYOR_1 = 6;
    public static final int INTAKE_CONVEYOR_2 = 7;

    // Launch
    public static final int LAUNCH_MOTOR_1 = 8;
    public static final int LAUNCH_MOTOR_2 = 9;

    // Drive
    public static final int DRIVE_MOTOR_R1 = 3;
    public static final int DRIVE_MOTOR_R2 = 1;
    public static final int DRIVE_MOTOR_L1 = 4;
    public static final int DRIVE_MOTOR_L2 = 2;

    // Climb
    public static final int CLIMB_TELESCOPE = 10;
    public static final int CLIMB_WINCH_1 = 11;
    public static final int CLIMB_WINCH_2 = 12;

    // Control Panel
    public static final int CP_ROTATE = 14;

    public static final int CP_ARM = 13;

    /* Buttons */
    // Driver controller
    public static final int DRIVER_CONTROLLER = 0;
    public static final int DRIVER_TURBO = 6; // top right
    public static final int DRIVER_SLOW = 5; // top left
    public static final int DRIVER_REVERSE = 2; // side button
    public static final int DRIVER_AUTO_ALIGN = 1; // trigger
    public static final int DRIVER_FIELD_ORIENT = 3; // bottom left
    public static final int DRIVER_ZERO = 4; // bottom right
    
    // Operator buttons
    public static final int OPERATOR_CONTROLLER = 1;
    public static final int OP_LAUNCH = 5; // left bumper
    public static final int OP_SLOW = 2; // B button
    public static final int OP_INTAKE = 6; // right bumper
    public static final int OP_LOCK_WINCH = 7; // menu left
    public static final int OP_RUN_WINCH = 8; // select right
    public static final int OP_REVERSE = 3; // X button
    public static final int OP_CONTROL_ARM = 1; // A button
    public static final int OP_TELESCOPES = 4; // Y button
    public static final int OP_REVERSE_TELE = 9; // left joy click
    public static final int OP_CONTROL_RIGHT = 10; // right joy click

    /* Sensors */
    // Intake sensors
    public static final int INTAKE_SENSOR_1 = 1;
    public static final int INTAKE_SENSOR_2 = 2;
    public static final int INTAKE_SENSOR_3 = 3;

    /* Limit Switches */
    public static final int ARM_LIMIT_UP = 5;
    public static final int ARM_LIMIT_DOWN = 6;

}
