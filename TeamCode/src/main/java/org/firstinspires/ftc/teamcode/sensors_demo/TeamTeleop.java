package org.firstinspires.ftc.teamcode.sensors_demo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.sensors_demo.TeamHardware;


@TeleOp(name="Teleop", group="GROUP6")
public class TeamTeleop extends LinearOpMode {

    private TeamHardware robot;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double leftX;
        double leftY;
        double rightX;
        double rightY;

        robot = new TeamHardware(hardwareMap,telemetry);
        telemetry.setAutoClear(false);
        robot.init(this);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (!gamepad1.atRest()) {
                    try {
                        leftX = Range.clip(gamepad1.left_stick_x, -1, 1);
                        leftY = Range.clip(gamepad1.left_stick_y, -1, 1);
                        rightX = Range.clip(gamepad1.right_stick_x, -1, 1);
                        rightY = Range.clip(gamepad1.right_stick_y, -1, 1);

                        if(leftX > 0.2){
                            telemetry.clear();
                            robot.sensors.GiveDistance();
                        }
                        else if(leftY > 0.2){
                            telemetry.clear();
                            robot.sensors.GiveTouch();
                        }
                    } catch (Exception e) {
                        telemetry.addData("TELEOP1:", "%s", e.toString());
                        telemetry.update();
                        RobotLog.ee("GROUP6", e, "TELEOP1");
                    }
                }


                // GAMEPAD 2 - LIFT AND GRAB: MANUAL & AUTO
                if (!gamepad2.atRest()) {
                    try {

                    } catch (Exception e) {
                        telemetry.addData("TELEOP2:", "%s", e.toString());
                        telemetry.update();
                        RobotLog.ee("GROUP6", e, "TELEOP2");
                    }
                }

                sleep(40);
            }
        }
    }
}