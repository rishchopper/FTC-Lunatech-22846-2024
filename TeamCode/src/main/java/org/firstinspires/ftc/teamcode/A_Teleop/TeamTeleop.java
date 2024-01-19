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
        String ARM = "PICKUP";

        double leftX1;
        double leftY1;
        double rightX1;
        double leftY2;
        double rightX2;

        double trigL2;
        double trigR2;
        double triggers;

        boolean a = false, b = false;

        boolean rTrig;
        boolean lTrig;

        long tA = System.currentTimeMillis();
        long tB = System.currentTimeMillis();


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


                        rTrig = gamepad2.right_bumper;
                        lTrig = gamepad2.left_bumper;

                        if (gamepad2.a && System.currentTimeMillis()-tA >= 500){
                            a = !a;
                            tA = System.currentTimeMillis();
                        }
                        if (gamepad2.b && System.currentTimeMillis()-tB >= 500){
                            b = !b;
                            tB = System.currentTimeMillis();
                        }

                        robot.launchPlane(gamepad2.dpad_down);
                        if (gamepad2.dpad_up){
                            robot.hangDeploy.setPosition(0.3);
                        }

                        if (gamepad2.dpad_left){
                            MODE = "ASSIST";
                        } else if(gamepad2.dpad_right){
                            MODE = "MANUAL CONTROL";
                        }

                        triggers = trigL2 + trigR2;

                        robot.setMotors(leftX1, -leftY1, rightX1);
                        robot.setLinearSlide(1, triggers);
                        robot.setLinearSlide(2, triggers);

                        if (a){
                            robot.grabLeft.setPosition(0.2);
                        } else {
                            robot.grabLeft.setPosition(0.32);
                        }
                        if (b){
                            robot.grabRight.setPosition(0.2);
                        } else {
                            robot.grabRight.setPosition(0.32);
                        }

                        if (MODE == "ASSIST"){
                            telemetry.addData("BOTMODE: ", "ASSISTED");
                            if (lTrig){
                                ARM = "PICKUP";
                                robot.clawRoll.setPosition(0.01);
                                robot.clawPitch.setPosition(0.57);
                                robot.armPos(0);
                            } else if (rTrig){
                                ARM = "DROP";
                                robot.armPos(1);
                                sleep(100);
                                robot.clawRoll.setPosition(0.67);
                                robot.clawPitch.setPosition(0.3);
                            }
                        } else {
                            telemetry.addData("BOTMODE: ", "MANUAL CONTROL");
                            robot.setManualArticulatedArm(leftY2);
                            if (lTrig){
                                ARM = "PICKUP";
                                robot.clawRoll.setPosition(0.01);
                                robot.clawPitch.setPosition(0.57);
                                robot.setManualArticulatedArm(leftY2);
                            } else if (rTrig){
                                ARM = "DROP";
                                robot.clawRoll.setPosition(0.67);
                                robot.clawPitch.setPosition(0.3);
                                robot.setManualArticulatedArm(leftY2);
                            }
                        }

                        telemetry.addData("ARM POS: ", ARM);
                        telemetry.addData("ARM ENCODER: ", robot.motorArticulatedArm.getCurrentPosition());

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