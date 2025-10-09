package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp (name="TesteMotor")
public class TesteMotor extends LinearOpMode{
    private DcMotorEx motorRight, motorLeft, motorIntake, motorCorreia;
    double power = 0.85;
    double intake_power = 0;
    double powercorreia = 0.5;
    int sentido = 1;
    @Override
    public void runOpMode(){
        motorRight = hardwareMap.get(DcMotorEx.class, "motorright");
        motorLeft  = hardwareMap.get(DcMotorEx.class, "motorleft");
        motorIntake = hardwareMap.get(DcMotorEx.class, "motorintake");
        motorCorreia = hardwareMap.get(DcMotorEx.class, "motorcorreia");
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.left_bumper){
                if(powercorreia > 0){
                    powercorreia -= 0.05;
                    sleep(200);
                }
            }else if(gamepad1.right_bumper){
                if(powercorreia < 1){
                    powercorreia += 0.05;
                    sleep(200);
                }
            }
            if(gamepad1.x) {
                sentido *= -1;
                sleep(300);
            }
            if(gamepad2.left_bumper){
                if(power > 0){
                    power -= 0.05;
                    sleep(200);
                }
            }else if(gamepad2.right_bumper){
                if(power < 1){
                    power += 0.05;
                    sleep(200);
                }
            }
            if(gamepad1.left_trigger != 0){
                intake_power = gamepad1.left_trigger;
                motorIntake.setPower(intake_power);
            }else if(gamepad1.right_trigger != 0){
                intake_power = -gamepad1.right_trigger;
                motorIntake.setPower(intake_power);
            }else{
                intake_power = 0;
                motorIntake.setPower(intake_power);
            }
            if(gamepad1.a){
                motorRight.setPower(power);
                motorLeft.setPower(-power);
            }else if(gamepad1.b){
                motorRight.setPower(-power);
                motorLeft.setPower(power);
            }else{
                motorRight.setPower(0);
                motorLeft.setPower(0);
            }
            telemetry.addData("power correia", powercorreia);
            telemetry.addData("power", power);
            telemetry.update();
            motorCorreia.setPower(sentido*powercorreia);
        }
    }
}
