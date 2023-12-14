package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name="CR_Servo_Rotate", group="Test")
public class cr_servo_rotate extends LinearOpMode {

    CRServo servo;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(CRServo.class, "servo");
        servo.setDirection(CRServo.Direction.REVERSE);

        waitForStart();

        servo.setPower(1.0);
        sleep(1500);
        servo.setPower(0.0);
    }
}