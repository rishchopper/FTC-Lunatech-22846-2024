package org.firstinspires.ftc.teamcode.sensors_demo;

import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TeamSensors {
    private DistanceSensor sensorDistance;
    private Rev2mDistanceSensor sensorTimeOfFlight;
    TouchSensor touchSensor;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    LinearOpMode myOpMode = null;
    ElapsedTime runtime;


    public TeamSensors(HardwareMap hwMap, Telemetry tmry, LinearOpMode opMd, ElapsedTime rntim) {
        hardwareMap = hwMap;
        telemetry = tmry;
        myOpMode = opMd;
        runtime = rntim;

        runtime = new ElapsedTime();
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");
    }

    public void init(LinearOpMode opmode) {
        myOpMode = opmode;
        sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
    }

    void GiveDistance()
    {
        try {
            telemetry.addData("deviceName", sensorDistance.getDeviceName() );
            telemetry.addData("Distance_Metre", String.format("%.01f m", sensorDistance.getDistance(DistanceUnit.METER)));
            telemetry.addData("Distance_Inch", String.format("%.01f in", sensorDistance.getDistance(DistanceUnit.INCH)));

            // Rev2mDistanceSensor specific methods.
//            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
//            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("setMotors", "%s", e.toString());
            telemetry.update();
            RobotLog.ee("GROUP6", e, "GiveDistance");
        }
    }

    void GiveTouch()
    {
        try {
            if (touchSensor.isPressed()) {
                telemetry.addData("Touch Sensor", "Is Pressed");
            } else {
                telemetry.addData("Touch Sensor", "Is Not Pressed");
            }

            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("GiveTouch", "%s", e.toString());
            telemetry.update();
            RobotLog.ee("GROUP6", e, "GiveTouch");
        }
    }
}
