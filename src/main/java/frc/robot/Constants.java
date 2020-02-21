
package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;

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


 //lol motr like constantly annoyed
public final class Constants {
    public static final int motorR1 = 3;
    public static final int motorR2 = 1;
    public static final int motorL1 = 4;
    public static final int motorL2 = 2;


  
    //buttons (NOTE: starts at 1 not 0)
    public static final int LEFTBUMPER = 5; 
    public static final int RIGHTBUMPER = 6; 
    public static final int XBOX_A = 1; 

    //controller stuff
    public static final int driver_joystick = 1;
    public static final int LOGITECH = 3; 
    public static final int driveYAxis = 1;



    //sensors


    public static final int zeroGyro = 4;
    public static final int fieldToggler = 3;
    public static final int pidToggler = 11;
    public static final int toggleSlow = 5;
    public static final int toggleTurbo = 6;
    public static final int toggleTarget = 7;
    public static final int toggleReverse = 8;
    public static final int TRIGGER = 1;
    public static final int SIDE = 2;

    public static final int INTAKE = 5;
    public static final int shooter1 = 8;
    public static final int shooter2 = 9;
    public static final int LOAD1 = 6;
    public static final int LOAD2 = 7;

    public static final int INTAKESENSOR1 = 1;
    public static final int INTAKESENSOR2 = 2;
    public static final int INTAKESENSOR3 = 3;

}
