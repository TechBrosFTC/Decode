package org.firstinspires.ftc.teamcode.deeptrack;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DriveTrain {
    public DcMotorEx leftFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightFront;
    public DcMotorEx rightBack;

    public DriveTrain(DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack) {
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
    }

    public void drive(double power, double steering) { //positive power = forward, negative power = backward, positive steering = right, negative steering = left
        leftFront.setPower(power + steering);
        leftBack.setPower(power + steering);
        rightFront.setPower(power - steering);
        rightBack.setPower(power - steering);
    }
    public void right(double power, double steering){
        leftFront.setPower((power + steering)*-1);
        leftBack.setPower((power - steering));
        rightFront.setPower((power + steering));
        rightBack.setPower((power - steering)*-1);
    }
    public void left(double power, double steering){
        leftFront.setPower((power + steering));
        leftBack.setPower((power - steering)*-1);
        rightFront.setPower((power + steering)*-1);
        rightBack.setPower((power - steering));
    }
    public void turn(double power) { //segue o ciclo trigonometrico
        leftFront.setPower(-power);
        leftBack.setPower(-power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
