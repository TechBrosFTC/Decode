package org.firstinspires.ftc.teamcode.deeptrack;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.DriveConstants;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**This is the main class for movimentation. There are methods for movimentation on xy axis and turns,
localization of the robot and the route calculation built-in in the functions*/
public class MecanumDrive {//Class mecanum drive
    public double x;//Set of variables for localization
    public double y;
    public double theta;
    public DriveTrain driveTrain;//Drive train
    private double kp;//pid constants
    private double kd;
    private double ki;
    private double direction;
    private double speed;
    private double target;
    private double proportional;
    private double derivative;
    private double integral;
    private double previousError;
    private double error;
    private RevHubOrientationOnRobot orientation;//orientation of the robot
    private double divisor = 100;
    public IMU imu;//Inertial measuremennt unit
    public Odometry odometry;//Odometry
    public MecanumDrive(double initial_x, double initial_y, double initial_theta, DriveTrain driveTrain, double kp, double kd, double ki, IMU imu, Odometry odometry){
        x = initial_x;
        y = initial_y;
        theta = initial_theta;
        this.driveTrain = driveTrain;
        this.kp = kp;
        this.kd = kd;
        this.ki = ki;
        this.odometry = odometry;
        this.imu = imu;
    }

    public void turn(double target, double max_speed, double min_speed){
        
    }
    /***
     Use power positivo para frente e power negativo para trás.
     @param distance Distance must be in centimeters
     @param theta_target This is the angle target that you want to follow during the move
     * */
    public void MRUVY(double min_speed, double max_speed, double distance, double theta_target){
        int ticks = (int) (distance / DriveConstants.CM_PER_TICK);
        double a = Math.abs(max_speed - min_speed)/(ticks*0.3);
        double b = Math.abs(max_speed);
        odometry.resetY();
        if (min_speed > 0) {
            while (odometry.getY() < ticks*0.7){
                error = (target - imu.getRobotYawPitchRollAngles().getYaw());
                proportional = error*kp;
                derivative = (error - previousError)*kd;
                integral = error+ki;
                previousError = error;
                direction = (proportional + derivative + integral)/divisor;
                driveTrain.drive(max_speed, direction);
            }
            while (odometry.getY() < ticks){
                error = (target - imu.getRobotYawPitchRollAngles().getYaw());
                proportional = error*kp;
                derivative = (error - previousError)*kd;
                integral = error+ki;
                previousError = error;
                direction = (proportional + derivative + integral)/divisor;
                speed = min_speed;
                driveTrain.drive(speed, direction);
            }
            driveTrain.stop();
        }else{
            while (odometry.getY() > -ticks*0.7) {
                error = (target - imu.getRobotYawPitchRollAngles().getYaw());
                proportional = error * kp;
                derivative = (error - previousError) * kd;
                integral = error + ki;
                previousError = error;
                direction = (proportional + derivative + integral)/divisor;
                driveTrain.drive(max_speed, direction);
            }
            while (odometry.getY() > -ticks) {
                error = (target - imu.getRobotYawPitchRollAngles().getYaw());
                proportional = error * kp;
                derivative = (error - previousError) * kd;
                integral = error + ki;
                previousError = error;
                direction = (proportional + derivative + integral)/divisor;
                speed = a * (this.odometry.getY() - (ticks*0.7)) + b;
                driveTrain.drive(min_speed, direction);
            }
            driveTrain.stop();
        }
    }
    /***
     Use power positivo para esquerda e power negativo para direita
     @param distance Distance must be in centimeters
     @param theta_target This is the angle target that you want to follow during the move
     */
    public void MRUVX(double min_speed, double max_speed, double distance,double theta_target){
        int ticks = (int) (distance / DriveConstants.CM_PER_TICK);
        double a = Math.abs(max_speed - min_speed)/(ticks*0.3);
        double b = Math.abs(max_speed);
        odometry.resetX();
        if(min_speed > 0){//se maior que vai para a esquerda, se menor que zero pra direita
            while (odometry.getX() > -ticks*0.7) {
                error = (imu.getRobotYawPitchRollAngles().getYaw() - target);
                proportional = error * kp;
                derivative = (error - previousError) * kd;
                integral = error + ki;
                previousError = error;
                direction = (proportional + derivative + integral)/divisor;
                driveTrain.left(max_speed, direction);
            }
            while (odometry.getX() > -ticks) {
                error = (imu.getRobotYawPitchRollAngles().getYaw()- target);
                proportional = error * kp;
                derivative = (error - previousError) * kd;
                integral = error + ki;
                previousError = error;
                direction = (proportional + derivative + integral)/divisor;
                speed = a * (this.odometry.getX() - (ticks*0.7)) + b;
                driveTrain.left(speed, direction);
            }
        }else{
            while (odometry.getX() > -ticks*0.7) {
                error = (imu.getRobotYawPitchRollAngles().getYaw() - target);
                proportional = error * kp;
                derivative = (error - previousError) * kd;
                integral = error + ki;
                previousError = error;
                direction = (proportional + derivative + integral)/divisor;
                driveTrain.right(max_speed*-1, direction);
            }
            while (odometry.getX() > -ticks) {
                error = (imu.getRobotYawPitchRollAngles().getYaw()- target);
                proportional = error * kp;
                derivative = (error - previousError) * kd;
                integral = error + ki;
                previousError = error;
                direction = (proportional + derivative + integral)/divisor;
                speed = a * (this.odometry.getX() - (ticks*0.7)) + b;
                driveTrain.right(speed*-1, direction);
            }
            driveTrain.stop();
        }
    }
    /**
     * @param target target in deegres, following the trigonometric cicle.
     * */
    public void curve(double target, double min_speed, double max_speed){
        double a = Math.abs(max_speed - min_speed)/(target*0.3);
        double b = Math.abs(max_speed);
        double sentido;
        imu.resetYaw();
        if(target > 0){
            sentido = 1;
        }else{
            sentido = -1;
        }
        while (Math.abs(imu.getRobotYawPitchRollAngles().getYaw()) < Math.abs(target)*0.7){
            driveTrain.turn(max_speed*sentido);
        }
        while (Math.abs(imu.getRobotYawPitchRollAngles().getYaw()) < Math.abs(target)){
            double speed = a*(imu.getRobotYawPitchRollAngles().getYaw()-target*0.7) + b;
            if(speed < min_speed){
                speed = min_speed;
            }
            driveTrain.turn(speed*sentido);
        }
        driveTrain.stop();
    }
    public void parar(){
        driveTrain.stop();
    }



}
