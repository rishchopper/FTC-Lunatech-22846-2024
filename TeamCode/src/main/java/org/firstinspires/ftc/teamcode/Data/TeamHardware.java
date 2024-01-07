package org.firstinspires.ftc.teamcode.Data;

import static java.lang.Math.abs;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TeamHardware {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    private ElapsedTime runtime;

    private LinearOpMode myOpMode = null;

    public DcMotorEx motorLeftFront;
    public DcMotorEx motorRightFront;
    public DcMotorEx motorLeftBack;
    public DcMotorEx motorRightBack;
    public DcMotorEx motorLinearSlideLeft;
    public DcMotorEx motorLinearSlideRight;
    public DcMotorEx motorArticulatedArm;
    public DcMotorEx motorPullup;

    public TouchSensor armExtreme;
    public TouchSensor armMean;
    public TouchSensor leftExtreme;
    public TouchSensor leftMean;
    public TouchSensor rightExtreme;
    public TouchSensor rightMean;

    public Servo grabLeft;
    public Servo grabRight;
    public CRServo planeLauncher;
    public Servo clawPitch;
    public Servo clawRoll;


    final double POWER_CHASSIS = 0.8;

    private double r, robotAngle, v1, v2, v3, v4;

    static final double COUNTS_PER_MOTOR_REV = 537.6898396; //Gobilda 5202 Motor Encoder 19.2:1	((((1+(46/17))) * (1+(46/11))) * 28)
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.937;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14158);
    static final double COUNTS_PER_DEGREE = 22 * (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.14158 * 90);


    public TeamHardware(HardwareMap hwMap, Telemetry tmry, LinearOpMode opMd) {
        hardwareMap = hwMap;
        telemetry = tmry;
        myOpMode = opMd;

        runtime = new ElapsedTime();
        motorLeftFront = hardwareMap.get(DcMotorEx.class, "motorLeftFront");
        motorRightFront = hardwareMap.get(DcMotorEx.class, "motorRightFront");
        motorRightBack = hardwareMap.get(DcMotorEx.class, "motorRightBack");
        motorLeftBack = hardwareMap.get(DcMotorEx.class, "motorLeftBack");
        motorLinearSlideLeft = hardwareMap.get(DcMotorEx.class, "motorLinearSlideLeft");
        motorLinearSlideRight = hardwareMap.get(DcMotorEx.class, "motorLinearSlideRight");
        motorArticulatedArm = hardwareMap.get(DcMotorEx.class, "motorArticulatedArm");
        planeLauncher = hardwareMap.get(CRServo.class, "planeLauncher");
        grabLeft = hardwareMap.get(Servo.class, "grabLeft");
        grabRight = hardwareMap.get(Servo.class, "grabRight");
        clawPitch = hardwareMap.get(Servo.class, "clawTilt");
        clawRoll = hardwareMap.get(Servo.class, "clawRoll");
    }

    public void init() {
        motorLeftFront.setDirection(DcMotorEx.Direction.FORWARD);
        motorLeftFront.setPower(0.0);

        motorRightFront.setDirection(DcMotorEx.Direction.REVERSE);
        motorRightFront.setPower(0.0);

        motorLeftBack.setDirection(DcMotorEx.Direction.FORWARD);
        motorLeftBack.setPower(0.0);

        motorRightBack.setDirection(DcMotorEx.Direction.REVERSE);
        motorRightBack.setPower(0.0);

        motorLinearSlideLeft.setDirection(DcMotorEx.Direction.FORWARD);
        motorLinearSlideLeft.setPower(0.0);

        motorLinearSlideRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorLinearSlideRight.setPower(0.0);
        
        motorArticulatedArm.setDirection(DcMotorEx.Direction.FORWARD);
        motorArticulatedArm.setPower(0.0);
    }

    /* Initialize standard Hardware interfaces */
    public void init_teleop(LinearOpMode opmode) {
        myOpMode = opmode;
        init();
        motorLeftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorLeftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorRightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorRightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorLeftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorRightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorRightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorLinearSlideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorLinearSlideLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorLinearSlideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorLinearSlideRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
        motorArticulatedArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorArticulatedArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        planeLauncher.setDirection(CRServo.Direction.REVERSE);
    }

    public void init_auto(LinearOpMode opmode) {
        myOpMode = opmode;
        init();
        motorLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorLeftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorRightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorLinearSlideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLinearSlideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorLinearSlideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLinearSlideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        
        motorArticulatedArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorArticulatedArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Starting at", "%7d: %7d: %7d: %7d",
                motorLeftFront.getCurrentPosition(),
                motorRightFront.getCurrentPosition(),
                motorLeftBack.getCurrentPosition(),
                motorRightBack.getCurrentPosition());
        telemetry.update();
    }



    public void setMotors(double x, double y, double rot)  //sets the motor speeds given an x, y and rotation value
    {
        try {
            r = Math.hypot(x, y);
            robotAngle = Math.atan2(y, x) - Math.PI / 4;

            v1 = r * Math.cos(robotAngle) + rot;
            v2 = r * Math.sin(robotAngle) - rot;
            v3 = r * Math.sin(robotAngle) + rot;
            v4 = r * Math.cos(robotAngle) - rot;


            motorLeftFront.setPower(v1 * POWER_CHASSIS);
            motorRightFront.setPower(v2 * POWER_CHASSIS);
            motorLeftBack.setPower(v3 * POWER_CHASSIS);
            motorRightBack.setPower(v4 * POWER_CHASSIS);
        } catch (Exception e) {
            telemetry.addData("setMotors", "%s", e.toString());
            telemetry.update();
            RobotLog.ee("LUNATECH", e, "setMotors");
        }
    }

    public void setManualArticulatedArm(double power){
        motorArticulatedArm.setPower(power);
    }

    public void setLinearSlide(int id, double pow){
        if (pow >= 0){
            switch (id){
                case 1:
                    motorLinearSlideLeft.setPower(pow);
                    break;

                case 2:
                    motorLinearSlideRight.setPower(pow);
            }
        }else if (pow < 0){
            switch (id){
                case 1:
                    motorLinearSlideLeft.setPower((pow*0.6));
                    break;

                case 2:
                    motorLinearSlideRight.setPower((pow*0.6));
            }
        }
    }

    public void launchPlane(boolean trigger){
        Runnable runnable = () -> {
            try {
                planeLauncher.setPower(1.0);
                sleep(1500);
                planeLauncher.setPower(0.0);
            } catch (InterruptedException e) {
                telemetry.addData("ERROR: ", e);
                telemetry.update();
            }
        };
        Thread plane = new Thread(runnable);
        if (trigger && !plane.isAlive()){
            plane.start();
        } else if (myOpMode.isStopRequested()){
            plane.interrupt();
            plane.destroy();
        }
    }

    public void encoderDrive(double speed, DataHolder.MOVEDIR dir, double distance,
                             double timeoutS) {

        // Ensure that the opmode is still active
        try {
            if (myOpMode.opModeIsActive()) {
                setChassisTargetPosition(dir, distance);
                beginChassisMotion(speed);
                moveChassisToTarget(dir, timeoutS);
                stopChassisAuto();
            }
        } catch (Exception e) {
            myOpMode.telemetry.addData("Exception encoderDrive", e.toString());
            myOpMode.telemetry.update();
            RobotLog.ee("LUNATECH", e, "exception in encoderDrive()");
        }
        myOpMode.sleep(40);   // optional pause after each move
    }

    public void encoderTurn(double speed, DataHolder.MOVEDIR dir, double degrees,
                            double timeoutS) {

        // Ensure that the opmode is still active
        try {
            if (myOpMode.opModeIsActive()) {


                setChassisTurnTargetPosition(dir, degrees);
                beginChassisMotion(speed);
                moveChassisToTarget(dir, timeoutS);
                stopChassisMotors();
            }
        } catch (Exception e) {
            myOpMode.telemetry.addData("Exception encoderTurn", e.toString());
            myOpMode.telemetry.update();
            RobotLog.ee("LUNATECH", e, "exception in encoderTurn()");
        }
        myOpMode.sleep(40);   // optional pause after each move
    }

    public void setChassisTurnTargetPosition(DataHolder.MOVEDIR dir, double degrees) {
        int targetPos;
        targetPos = (int) (degrees * COUNTS_PER_DEGREE);
        try {
            switch (dir) {
                case ROTATE_RIGHT:
                    motorLeftFront.setTargetPosition(motorLeftFront.getCurrentPosition() + targetPos);
                    motorRightFront.setTargetPosition(motorRightFront.getCurrentPosition() - targetPos);
                    motorLeftBack.setTargetPosition(motorLeftBack.getCurrentPosition() + targetPos);
                    motorRightBack.setTargetPosition(motorRightBack.getCurrentPosition() - targetPos);
                    break;
                case ROTATE_LEFT:
                    motorLeftFront.setTargetPosition(motorLeftFront.getCurrentPosition() - targetPos);
                    motorRightFront.setTargetPosition(motorRightFront.getCurrentPosition() + targetPos);
                    motorLeftBack.setTargetPosition(motorLeftBack.getCurrentPosition() - targetPos);
                    motorRightBack.setTargetPosition(motorRightBack.getCurrentPosition() + targetPos);
                    break;
                default:
                    motorLeftFront.setTargetPosition(motorLeftFront.getCurrentPosition());
                    motorRightFront.setTargetPosition(motorRightFront.getCurrentPosition());
                    motorLeftBack.setTargetPosition(motorLeftBack.getCurrentPosition());
                    motorRightBack.setTargetPosition(motorRightBack.getCurrentPosition());
            }
        } catch (Exception e) {
            myOpMode.telemetry.addData("Exception setChassisTurnTargetPosition", e.toString());
            myOpMode.telemetry.update();
            RobotLog.ee("LUNATECH", e, "exception in setChassisTurnTargetPosition()");
        }

    }

    void beginChassisMotion(double speed) {
        // Turn On RUN_TO_POSITION
        try {
            motorLeftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motorRightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motorLeftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motorRightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorLeftFront.setPower(abs(speed));
            motorRightFront.setPower(abs(speed));
            motorLeftBack.setPower(abs(speed));
            motorRightBack.setPower(abs(speed));
        } catch (Exception e) {
            myOpMode.telemetry.addData("Exception beginChassisMotion", e.toString());
            myOpMode.telemetry.update();
            RobotLog.ee("LUNATECH", e, "exception in beginChassisMotion()");
        }
    }

    void moveChassisToTarget(DataHolder.MOVEDIR dir, double timeoutS) {
        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.

        try {
            while (myOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorLeftFront.isBusy() || motorRightFront.isBusy() ||
                            motorLeftBack.isBusy() || motorRightBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", motorLeftFront.getTargetPosition(),
                        motorRightFront.getTargetPosition());
                telemetry.addData("Path2", "%s: Now at %7d :%7d: %7d: %7d",
                        dir.name(),
                        motorLeftFront.getCurrentPosition(),
                        motorRightFront.getCurrentPosition(),
                        motorLeftBack.getCurrentPosition(),
                        motorRightBack.getCurrentPosition());
                myOpMode.telemetry.update();
                myOpMode.sleep(40);
            }
        } catch (Exception e) {
            myOpMode.telemetry.addData("Exception moveChassisToTarget", e.toString());
            myOpMode.telemetry.update();
            RobotLog.ee("LUNATECH", e, "exception in moveChassisToTarget()");
        }
    }

    public void setChassisTargetPosition(DataHolder.MOVEDIR dir, double distance) {
        int targetPos;
        targetPos = (int) (distance * COUNTS_PER_INCH);

        try {
            switch (dir) {
                case FRONT:
                    motorLeftFront.setTargetPosition(motorLeftFront.getCurrentPosition() + targetPos);
                    motorRightFront.setTargetPosition(motorRightFront.getCurrentPosition() + targetPos);
                    motorLeftBack.setTargetPosition(motorLeftBack.getCurrentPosition() + targetPos);
                    motorRightBack.setTargetPosition(motorRightBack.getCurrentPosition() + targetPos);
                    break;
                case BACK:
                    motorLeftFront.setTargetPosition(motorLeftFront.getCurrentPosition() - targetPos);
                    motorRightFront.setTargetPosition(motorRightFront.getCurrentPosition() - targetPos);
                    motorLeftBack.setTargetPosition(motorLeftBack.getCurrentPosition() - targetPos);
                    motorRightBack.setTargetPosition(motorRightBack.getCurrentPosition() - targetPos);
                    break;
                case RIGHT:
                    motorLeftFront.setTargetPosition(motorLeftFront.getCurrentPosition() + targetPos);
                    motorRightFront.setTargetPosition(motorRightFront.getCurrentPosition() - targetPos);
                    motorLeftBack.setTargetPosition(motorLeftBack.getCurrentPosition() - targetPos);
                    motorRightBack.setTargetPosition(motorRightBack.getCurrentPosition() + targetPos);
                    break;
                case LEFT:
                    motorLeftFront.setTargetPosition(motorLeftFront.getCurrentPosition() - targetPos);
                    motorRightFront.setTargetPosition(motorRightFront.getCurrentPosition() + targetPos);
                    motorLeftBack.setTargetPosition(motorLeftBack.getCurrentPosition() + targetPos);
                    motorRightBack.setTargetPosition(motorRightBack.getCurrentPosition() - targetPos);
                    break;
                case BACK_LEFT:
                    motorLeftFront.setTargetPosition(motorLeftFront.getCurrentPosition() - targetPos);
                    motorRightFront.setTargetPosition(motorRightFront.getCurrentPosition());
                    motorLeftBack.setTargetPosition(motorLeftBack.getCurrentPosition());
                    motorRightBack.setTargetPosition(motorRightBack.getCurrentPosition() - targetPos);
                    break;
                case FRONT_LEFT:
                    motorLeftFront.setTargetPosition(motorLeftFront.getCurrentPosition());
                    motorRightFront.setTargetPosition(motorRightFront.getCurrentPosition() + targetPos);
                    motorLeftBack.setTargetPosition(motorLeftBack.getCurrentPosition() + targetPos);
                    motorRightBack.setTargetPosition(motorRightBack.getCurrentPosition());
                    break;
                case BACK_RIGHT:
                    motorLeftFront.setTargetPosition(motorLeftFront.getCurrentPosition());
                    motorRightFront.setTargetPosition(motorRightFront.getCurrentPosition() - targetPos);
                    motorLeftBack.setTargetPosition(motorLeftBack.getCurrentPosition() - targetPos);
                    motorRightBack.setTargetPosition(motorRightBack.getCurrentPosition());
                    break;
                case FRONT_RIGHT:
                    motorLeftFront.setTargetPosition(motorLeftFront.getCurrentPosition() + targetPos);
                    motorRightFront.setTargetPosition(motorRightFront.getCurrentPosition());
                    motorLeftBack.setTargetPosition(motorLeftBack.getCurrentPosition());
                    motorRightBack.setTargetPosition(motorRightBack.getCurrentPosition() + targetPos);
                    break;
                default:
                    motorLeftFront.setTargetPosition(motorLeftFront.getCurrentPosition());
                    motorRightFront.setTargetPosition(motorRightFront.getCurrentPosition());
                    motorLeftBack.setTargetPosition(motorLeftBack.getCurrentPosition());
                    motorRightBack.setTargetPosition(motorRightBack.getCurrentPosition());
            }
        } catch (Exception e) {
            myOpMode.telemetry.addData("Exception setChassisTargetPosition", e.toString());
            myOpMode.telemetry.update();
            RobotLog.ee("LUNATECH", e, "exception in setChassisTargetPosition()");
        }
    }

    public void stopChassisMotors() {
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
    }

    void stopChassisAuto() {
        // Stop all motion;
        try {
            motorLeftFront.setPower(0);
            motorRightFront.setPower(0);
            motorLeftBack.setPower(0);
            motorRightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            motorLeftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motorRightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motorLeftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motorRightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            myOpMode.telemetry.addData("Exception stopChassis", e.toString());
            myOpMode.telemetry.update();
            RobotLog.ee("LUNATECH", e, "exception in stopChassis()");
        }
    }
}