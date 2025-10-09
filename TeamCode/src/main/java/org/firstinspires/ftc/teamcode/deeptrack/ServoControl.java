package org.firstinspires.ftc.teamcode.deeptrack;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
/**
 * This class is used to control the servos.
 * @author Gabriel Borges*/
public class ServoControl {
    public Servo servo;
    public ServoControl(HardwareMap hardwareMap, String servo_name, boolean revert){
        servo = hardwareMap.get(Servo.class, servo_name);
        if (revert){
            servo.setDirection(Servo.Direction.REVERSE);
        }
    }
}