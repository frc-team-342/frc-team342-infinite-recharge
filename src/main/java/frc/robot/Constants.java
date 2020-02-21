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
    public static final int intake_primary = 5;
    public static final int intake_conveyor1 = 6;
    public static final int intake_conveyor2 = 7;

    // Launch
    public static final int launch_motor1 = 8;
    public static final int launch_motor2 = 9;

    // Drive
    public static final int drive_motorR1 = 3;
    public static final int drive_motorR2 = 1;
    public static final int drive_motorL1 = 4;
    public static final int drive_motorL2 = 2;

    // Climb
    public static final int climb_telescope = 10;
    public static final int climb_winch1 = 11;
    public static final int climb_winch2 = 12;

    // Control Panel
    public static final int cp_rotate = 9;
    public static final int cp_arm = 10;

    /* Buttons */
    // Driver controller
    public static final int driver_controller = 0;
    public static final int driver_autoAlign = 1;
    public static final int driver_fieldOrient = 2;
    public static final int driver_turbo = 5;

    // Operator buttons
    public static final int operator_controller = 1;
    public static final int op_launch = 4;
    public static final int op_slow = 5;
    public static final int op_intake = 6;
    public static final int op_lockWinch = 7;
    public static final int op_runWinch = 8;

    /* Sensors */
    // Intake sensors
    public static final int intake_sensor1 = 0;
    public static final int intake_sensor2 = 1;
    public static final int intake_sensor3 = 2;
    public static final int intake_sensor4 = 3;
    public static final int intake_sensor5 = 4;

    /*Limit Switches*/
    public static final int ARMLIMITUP = 5;
    public static final int ARMLIMITDOWN = 6;
}
