package org.firstinspires.ftc.teamcode.sensors_demo;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeamSensors;

public class TeamHardware {
    public TeamSensors sensors;

    HardwareMap hardwareMap;
    Telemetry telemetry;
    private ElapsedTime runtime;

    private LinearOpMode myOpMode = null;

    public TeamHardware(HardwareMap hwMap, Telemetry tmry) {
        hardwareMap = hwMap;
        telemetry = tmry;
        runtime = new ElapsedTime();
    }

    /* Initialize standard Hardware interfaces */
    public void init(LinearOpMode opmode) {
        myOpMode = opmode;
        sensors = new TeamSensors(hardwareMap,telemetry,myOpMode,runtime);
        sensors.init(myOpMode);
    }



}