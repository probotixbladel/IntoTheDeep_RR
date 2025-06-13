package org.firstinspires.ftc.teamcode.drive;

public class Constants {
    public static final double DEFAULT_GEARSHIFT = 0.25;
    public static final double DEFAULT_TURNSPEED = 0.4;
    public static final double DEFAULT_POW = 2.3;
    public static final double DEFAULT_KS = 0.4;

    // GameObjectController constants

    //lift:
    public static final double LIFT_POSITION = 380;
    public static final double TOP_LIFT = 2650;

    public static final double POSITION_LEFT = 0.22; //position van wat?
    public static final double POSITION_RIGHT = 0.80; //position van wat?
 
    
    public static final boolean TARGET = false;
    public static final double TICKS_IN_DEGREE = 450 / 180.0;
    public static final boolean GRAB = false;
    public static final double GRAB_POSITION = 0.5;

    // slidout arm:
    public static final double SLIDEOUT_ARM_SPEED = 10.0;
    public static final int SLIDEOUT_ARM_START_POSITION = 55; //start position? voorheen: ERECTION
    public static final int SLIDEOUT_ARM_END_POSITION = 65; //end position? voorheen: FINISH_ERECTION
    public static final double SLIDEOUT_P = 0.02;
    public static final double SLIDEOUT_I = 0.008;
    public static final double SLIDEOUT_D = 0.001;

    // output arm:
    public static final double OUTPUT_ARM_START_POSITION = 0; // start position? voorheen: GOTOHEIL
    public static final double OUTPUT_ARM_TOP_POSITION = 500; // top position? voorheen: TOPHEIL
    public static final double OUTPUT_P = 0.0028;
    public static final double OUTPUT_I = 0.0013;
    public static final double OUTPUT_D = 0.00035;



}