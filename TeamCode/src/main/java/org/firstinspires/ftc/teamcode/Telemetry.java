package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp (name = "telemetry")
public class Telemetry extends LinearOpMode {
    DcMotorEx rightFront;
    DcMotorEx rightBack;
    DcMotorEx leftBack;
    DcMotorEx leftFront;
    DcMotorEx par1;
    DcMotorEx perp;
    DcMotorEx encoder;
    IMU imu;
    // MecanumDrive mecanumDrive = new MecanumDrive(0, 0, 0, direitaFrente, direitaTras, esquerdaFrente, esquerdaTras, 0, 0, 0);
    public void runOpMode(){
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        encoder = hardwareMap.get(DcMotorEx.class, "encoderseletor");
        par1 = hardwareMap.get(DcMotorEx.class, "par");
        imu = hardwareMap.get(IMU.class, "imu");
        perp = hardwareMap.get(DcMotorEx.class, "perp");
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        RobotSetup peach = new RobotSetup(hardwareMap, rightFront, rightBack, leftFront, leftBack, par1, perp);
        peach.mecanumDrive.odometry.resetY();
        peach.mecanumDrive.odometry.resetX();
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("odometria y: ", peach.mecanumDrive.odometry.getY());
            telemetry.addData("odometria x: ", peach.mecanumDrive.odometry.getX());
            telemetry.addData("encoder", encoder.getCurrentPosition());
            telemetry.addData("giro: ", peach.imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.update();
        }
    }
}
