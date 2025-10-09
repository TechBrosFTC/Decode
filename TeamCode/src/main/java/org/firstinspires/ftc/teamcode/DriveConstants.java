package org.firstinspires.ftc.teamcode;

public class DriveConstants {
    /**Constants for drive calculations
     We decided to use the SI system for consistency. @author Gabriel Borges */
    public static final double ODOMETRY_WHEEL_DIAMETER = 3.6; // in centimeters
    public static final double TRACK_WIDTH = 0; // distance between the odometry wheels in centimeters
    public static final double MOTOR_TICKS_PER_REV = 8192; // You can find this data in the specifications of your motor
    public static final double GEAR_RATIO =  1/15; //Some motor have a built-in gear ratio that you need to take into account, you can find this data in the specifications of your motor
    public static final double MOTOR_MAX_RPM = 1; // You can find this data in the specifications of your motor
    public static final double MAXIMUM_DRIVE_SPEED = MOTOR_MAX_RPM * GEAR_RATIO * Math.PI * ODOMETRY_WHEEL_DIAMETER / 3600; //Don't change this formule unless you know what you are doing, this is the teoretically max acceleraccion
    public static final double CM_PER_TICK = (ODOMETRY_WHEEL_DIAMETER * Math.PI) / (MOTOR_TICKS_PER_REV);
    public static final double KP = 0.001; // Proportional constant for PID controller
    public static final double KI = 0.001; // Integral constant for PID controller
    public static final double KD = 0.001; // Derivative constant for PID controller
    public static final double ROBOT_MASS = 0; //In kilograms
    public static final double ROBOT_LENGHT = 0;//In centimeters
    public static final double ROBOT_WIDTH = 0;//In centimeters

}
