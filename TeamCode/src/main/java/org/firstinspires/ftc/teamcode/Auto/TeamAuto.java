package org.firstinspires.ftc.teamcode.Auto;


import static org.firstinspires.ftc.teamcode.Data.DataHolder.MOVEDIR.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Data.DataHolder;
import org.firstinspires.ftc.teamcode.Data.TeamHardware;

@Autonomous(name="Auto", group="GROUP1")
public class TeamAuto extends LinearOpMode {

    private TeamHardware robot;
    private DataHolder.MOVEDIR signal;

    @Override
    public void runOpMode() {
        robot = new TeamHardware(hardwareMap, telemetry, this);
        telemetry.setAutoClear(false);
        robot.init_auto(this);

        waitForStart();

        robot.encoderDrive(1, FRONT, 12, 8);
        robot.encoderDrive(1, RIGHT, 12, 8);
        robot.encoderDrive(1, BACK, 12, 8);
        robot.encoderDrive(1, BACK, 12, 8);
    }
}
