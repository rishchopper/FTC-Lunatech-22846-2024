package org.firstinspires.ftc.teamcode.sensors_demo;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto", group="GROUP6")
public class TeamAuto extends LinearOpMode {

    private TeamHardware robot;
    private DataHolder.MOVEDIR signal;

    @Override
    public void runOpMode() {
        robot = new TeamHardware(hardwareMap, telemetry);
        telemetry.setAutoClear(false);
        robot.init(this);

        waitForStart();
        telemetry.clear();
        robot.sensors.GiveDistance();
        sleep(3000);
        telemetry.clear();
        robot.sensors.GiveTouch();
        sleep(3000);
        telemetry.clear();
    }
}
