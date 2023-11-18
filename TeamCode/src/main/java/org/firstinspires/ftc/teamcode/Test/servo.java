package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Servo Tester Swing", group="Test")
public class servo extends LinearOpMode {

    Servo servo;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "servo");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            servo.setPosition(1.0);
            sleep (5000);
            servo.setPosition(0.0);
            sleep(5000);
        }
    }
}