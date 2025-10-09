package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.deeptrack.DriveTrain;
import org.firstinspires.ftc.teamcode.deeptrack.MecanumDrive;
import org.firstinspires.ftc.teamcode.deeptrack.Odometry;

/**
 *This is the class where the robot setup is initialized. You need to make sure you assigned the right names
 * to the motors and sensors. Select the right kind of odometry you have.
 * @author Gabriel Borges
 */
public class RobotSetup {
    public DcMotorEx leftFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightFront;
    public DcMotorEx rightBack;
    public DcMotorEx par1;
    public DcMotorEx par2;
    public DcMotorEx perp;
    public IMU imu;
    public RobotSetup(HardwareMap hardwareMap, DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, DcMotorEx par1, DcMotorEx par2, DcMotorEx perp){
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
        odometry = new Odometry(0, 0, 0, par1, par2, perp);
        driveTrain = new DriveTrain(leftFront, leftBack, rightFront, rightBack);
        imu = hardwareMap.get(IMU.class, "imu");
        orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        imu.initialize(new IMU.Parameters(orientation));
        mecanumDrive = new MecanumDrive(0, 0, 0, driveTrain, 2.2, 2, 0, imu, odometry );
    }
    public RobotSetup(HardwareMap hardwareMap, DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, DcMotorEx par1, DcMotorEx perp){
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
        this.par1 = par1;
        this.par2 = null;
        this.perp = perp;
        odometry = new Odometry(0, 0, 0, par1, perp);
        driveTrain = new DriveTrain(leftFront, leftBack, rightFront, rightBack);
        imu = hardwareMap.get(IMU.class, "imu");
        orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientation));
        mecanumDrive = new MecanumDrive(0, 0, 0, driveTrain, 2, 0, 0, imu, odometry );
    }
    /** Set the encoders and the odometry. If your robot does not fit 3 odometrys, delete the par 2 line and the par 2 argument on the odometry instance.*/
    public Odometry odometry;
    public DriveTrain driveTrain;
    public RevHubOrientationOnRobot orientation;
    public MecanumDrive mecanumDrive;
}
