package org.firstinspires.ftc.teamcode.A_Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Data.TeamHardware;


@TeleOp(name="Teleop", group="GROUP1")
public class TeamTeleop extends LinearOpMode {

    private TeamHardware robot;

    @Override
    public void runOpMode() {
        double leftX1;
        double leftY1;
        double rightX1;

        double trigL1;
        double trigR1;
        double triggers;

        robot = new TeamHardware(hardwareMap, telemetry, this);
        telemetry.setAutoClear(false);
        robot.init_teleop(this);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive() && !isStopRequested()) {
                //if (!gamepad1.atRest()) {
                    try {
                        leftX1 = Range.clip(gamepad1.left_stick_x, -1, 1);
                        trigL1 = -Range.clip(gamepad1.left_trigger, -1, 1);
                        trigR1 = Range.clip(gamepad1.right_trigger, -1, 1);
                        leftY1 = Range.clip(gamepad1.left_stick_y, -1, 1);
                        rightX1 = Range.clip(gamepad1.right_stick_x, -1, 1);

                        triggers = trigL1 + trigR1;

                        robot.setMotors(leftX1, -leftY1, rightX1);
                        robot.setLinearSlide(1, triggers);
                        robot.setLinearSlide(2, triggers);

                        telemetry.clear();
                        telemetry.addData("GAMEPAD1", "Front %f,  Right %f, Turn %f", leftY1, leftX1, rightX1);
                        telemetry.addData("Triggers", "Left_Trig %f, Right_Trig %f, Trig %f", trigL1, trigR1, triggers);
                        telemetry.addData("Velocty", "Right_Back %f, Left_Front %f", robot.motorRightBack.getVelocity(), robot.motorLeftFront.getVelocity());
                        telemetry.update();
                    } catch (Exception e) {
                        telemetry.addData("TELEOP1:", "%s", e.toString());
                        telemetry.update();
                        RobotLog.ee("SMTECH", e, "TELEOP 1");
                    }
                //}
                //else{
                //      robot.stopChassisMotors();
                //}
                sleep(40);
            }
        }
    }
}