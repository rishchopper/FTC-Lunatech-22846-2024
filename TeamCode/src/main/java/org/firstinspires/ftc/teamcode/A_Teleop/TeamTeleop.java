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
        String MODE = "ASSIST";

        double leftX1;
        double leftY1;
        double rightX1;
        double leftY2;
        double rightX2;

        double trigL2;
        double trigR2;
        double triggers;

        boolean dpadUP2;
        boolean dpadDOWN2;
        boolean dpadLEFT2;
        boolean dpadRIGHT2;

        double rTrig;
        float lTrig;

        robot = new TeamHardware(hardwareMap, telemetry, this);
        telemetry.setAutoClear(true);
        robot.init_teleop(this);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive() && !isStopRequested()) {
                //if (!gamepad1.atRest()) {
                    try {
                        trigL2 = -Range.clip(gamepad2.left_trigger, -1, 1);
                        trigR2 = Range.clip(gamepad2.right_trigger, -1, 1);

                        leftX1 = Range.clip(gamepad1.left_stick_x, -1, 1);
                        leftY1 = Range.clip(gamepad1.left_stick_y, -1, 1);
                        rightX1 = Range.clip(gamepad1.right_stick_x, -1, 1);

                        leftY2 = Range.clip(gamepad2.left_stick_y, -1, 1);
                        rightX2 = Range.clip(gamepad2.right_stick_x, -1, 1);

                        dpadUP2 = gamepad2.dpad_up;
                        dpadDOWN2 = gamepad2.dpad_down;
                        dpadLEFT2 = gamepad2.dpad_left;
                        dpadRIGHT2 = gamepad2.dpad_right;

                        rTrig = gamepad2.right_trigger;
                        lTrig = gamepad2.right_trigger;


                        robot.launchPlane(dpadDOWN2);
                        if (dpadUP2){
                            robot.hangDeploy.setPosition(0.3);
                        }

                        if (dpadLEFT2){
                            MODE = "ASSIST";
                        } else if(dpadRIGHT2){
                            MODE = "MANUAL CONTROL";
                        }

                        triggers = trigL2 + trigR2;

                        robot.setMotors(leftX1, -leftY1, rightX1);
                        robot.setLinearSlide(1, triggers);
                        robot.setLinearSlide(2, triggers);

                        if (MODE == "ASSIST"){
                            telemetry.addData("BOTMODE: ", "ASSISTED");
                            if (lTrig > 0.2){
                                telemetry.addData("ARM MODE: ", "PICKUP");
                                robot.clawRoll.setPosition(0.0);
                                robot.clawPitch.setPosition(0.0);
                                robot.armPos(0);
                            } else if (rTrig > 0.2){
                                telemetry.addData("ARM MODE: ", "DROP");
                                robot.clawRoll.setPosition(0.6);
                                robot.clawPitch.setPosition(0.4);
                                robot.armPos(1);
                            }
                        } else {
                            telemetry.addData("BOTMODE: ", "MANUAL CONTROL");
                            robot.setManualArticulatedArm(leftY2);
                            if (lTrig > 0.2){
                                telemetry.addData("CLAW MODE: ", "PICKUP");
                                robot.clawRoll.setPosition(0.0);
                                robot.clawPitch.setPosition(0.0);
                            } else if (rTrig > 0.2){
                                telemetry.addData("CLAW MODE: ", "DROP");
                                robot.clawRoll.setPosition(0.6);
                                robot.clawPitch.setPosition(0.4);
                            }
                        }

                        telemetry.addData("GAMEPAD1", "Front %f,  Right %f, Turn %f", leftY1, leftX1, rightX1);
                        telemetry.addData("Triggers", "Left_Trig %f, Right_Trig %f, Trig %f", trigL2, trigR2, triggers);
                        telemetry.addData("Velocty", "Left %f, Right %f", robot.motorLinearSlideLeft.getVelocity(), robot.motorLinearSlideRight.getVelocity());
                        telemetry.update();
                    } catch (Exception e) {
                        telemetry.addData("TELEOP:", "%s", e.toString());
                        telemetry.update();
                        RobotLog.ee("LUNATECH", e, "TELEOP");
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