package frc.robot;

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

    /* Motors */
    // Intake
    public static final int INTAKE_PRIMARY = 5;
    public static final int INTAKE_CONVEYOR1 = 6;
    public static final int INTAKE_CONVEYOR2 = 7;

    // Launch
    public static final int LAUNCH_MOTOR1 = 8;
    public static final int LAUNCH_MOTOR2 = 9;

    // Drive
    public static final int DRIVE_MOTOR_R1 = 3;
    public static final int DRIVE_MOTOR_R2 = 1;
    public static final int DRIVE_MOTOR_L1 = 4;
    public static final int DRIVE_MOTOR_L2 = 2;

    // Climb
    public static final int CLIMB_TELESCOPE = 10;
    public static final int CLIMB_WINCH1 = 11;
    public static final int CLIMB_WINCH2 = 12;

    // Control Panel
    public static final int CP_ROTATE = 9;
    public static final int CP_ARM = 13;

    /* Buttons */
    // Driver controller
    public static final int DRIVER_CONTROLLER = 0;
    public static final int DRIVER_AUTOALIGN = 1;
    public static final int DRIVER_FIELDORIENT = 2;
    public static final int DRIVER_TURBO = 5;

    // Operator buttons
    public static final int OPERATOR_CONTROLLER = 1;
    public static final int OP_LAUNCH = 4;
    public static final int OP_SLOW = 5;
    public static final int OP_INTAKE = 6;
    public static final int OP_LOCKWINCH = 7;
    public static final int OP_RUNWINCH = 8;

    /* Sensors */
    // Intake sensors
    public static final int INTAKE_SENSOR1 = 0;
    public static final int INTAKE_SENSOR2 = 1;
    public static final int INTAKE_SENSOR3 = 2;
    public static final int INTAKE_SENSOR4 = 3;
    public static final int INTAKE_SENSOR5 = 4;

}
