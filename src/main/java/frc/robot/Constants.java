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
    public static final int CP_ROTATE = 9;
    public static final int CP_ARM = 10;

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
    public static final int OP_CONTROL_LEFT = 9; // left joy click
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
