package org.firstinspires.ftc.teamcode.deeptrack;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Odometry {
    public double x;
    public double y;
    public double theta;
    private int odometryType = 0;
    private DcMotorEx par1;
    private DcMotorEx par2;
    private DcMotorEx perp;
    public Odometry(double initial_x, double initial_y, double initial_theta, DcMotorEx par1, DcMotorEx par2, DcMotorEx perp) {
        x = initial_x;
        y = initial_y;
        theta = initial_theta;
        this.par1 = par1;
        this.par2 = par2;
        this.perp = perp;
        odometryType = 0;
    }
    public Odometry(double initial_x, double initial_y, double initial_theta, DcMotorEx par1, DcMotorEx perp){
        x = initial_x;
        y = initial_y;
        theta = initial_theta;
        this.par1 = par1;
        this.par2 = null;
        this.perp = perp;
        odometryType = 1;
    }
    public void resetX(){
        perp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        x = 0;
    }
    public void resetY(){
        switch (odometryType) {
            case 0:
                par1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                par1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                par2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                par2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            case 1:
                par1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                par1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
        }
    }
    public double getY(){
        double value;
        switch (odometryType) {
            case 0:
                value = Math.round((par2.getCurrentPosition()+ par1.getCurrentPosition()) / 2);
                break;
            case 1:
                value = par1.getCurrentPosition();
                break;
            default:
                value = 0;
                break;
        }
        return value;
    }
    public double getX() {
        return perp.getCurrentPosition();
    }
}
