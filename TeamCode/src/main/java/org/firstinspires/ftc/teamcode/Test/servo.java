package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Servo Tester Swing", group="Test")
public class servo extends LinearOpMode {

    CRServo servo;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(CRServo.class, "servo");
        servo.setDirection(DcMotorSimple.Direction.FORWARD );

        waitForStart();

        servo.setPower(1.0);
        sleep(1000);
        servo.setPower(0.0);
    }
}