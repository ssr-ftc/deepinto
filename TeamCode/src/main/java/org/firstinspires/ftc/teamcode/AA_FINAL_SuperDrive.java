package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

//aryan
/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
@Config

public class AA_FINAL_SuperDrive extends LinearOpMode {

    private Servo servo_drone;
    private Servo servo_claw;
    private Servo servo_right;
    private Servo servo_left;
    private DcMotor motor_lever;
    private DcMotor linear_slide;


    int motor_lever_pos;
    double speed;
    int motor_lever_pos_NEW;
    boolean rigging = false;

    PIDFController slidecontroller; //for pid control

    PIDCoefficients slidecoeffes; //for pid control

    boolean toslidreset = false;

//    public static double hangvalue = -400;

    public static double slidezero = -100;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double tgtpowerls = 100;

    static enum slidereststages{
        correction,
        reset,
        normal,

        zero
    }

    boolean leftdelay = false;

    boolean rightdelay = false;

    slidereststages slidereststage =   slidereststages.correction;

    SampleMecanumDrive drive;

    double limitspeed;

    @Override
    public void runOpMode() throws InterruptedException {
         drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        servo_claw = hardwareMap.get(Servo.class, "servo_claw");
        servo_drone = hardwareMap.get(Servo.class, "servo_drone");
        servo_right = hardwareMap.get(Servo.class, "servo_right");
        servo_left = hardwareMap.get(Servo.class, "servo_left");
        motor_lever = hardwareMap.get(DcMotor.class, "motor_lever");
        linear_slide = hardwareMap.get(DcMotor.class, "linear_slide");

        //drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        slidecoeffes = new PIDCoefficients(0.025, 0, 0); //for pid control
        // create the controller
        slidecontroller = new PIDFController(slidecoeffes); //for pid control


        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        waitForStart();

        while (!isStopRequested()) {

            set_auto_power();

            if (gamepad1.a) {
                speed = 0.3;
            } else {
                speed = 1;
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * speed,
                            (-gamepad1.right_trigger + gamepad1.left_trigger) * speed * (limitspeed*0.8),
                            -gamepad1.right_stick_x * speed * (limitspeed*0.7)
                    )

            );


//
            if (rigging) {
                pidhang(-400);
            }

            drive.update();

            hanging();
            use_drone_luancher_modified();
            gamepad22();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("lever position", motor_lever.getCurrentPosition());
            telemetry.update();
        }


        if(isStopRequested()){
            drive.setMotorPowers(0,0,0,0);
        }
    }


    private void move_left_and_right_claw() {
        if (gamepad2.a) {
            rightClamp();
//            leftClamp();
        } else if (gamepad2.b) {
            rightRelease();
//            leftRelease();
        }
        if (gamepad2.dpad_left){
            clawHorizontal();
        } else if (gamepad2.dpad_down) {
            clawVertical();
        }
        telemetry.addData("claw position", 1*servo_right.getPosition());
        telemetry.update();
        if (gamepad2.left_trigger > 0.5 && !leftdelay) {
            if(gamepad2.x) {
//                leftClamp();

            }
            else {
//                leftRelease();

            }
            leftdelay = true;

        }
        else if (!(gamepad2.left_trigger > 0.5) && !gamepad2.x && leftdelay) {
            leftdelay = false;
        }

        if (gamepad2.right_trigger > 0.5 && !rightdelay) {
            if(gamepad2.x) {
                rightClamp();
            }
            else{
                rightRelease();

            }
            rightdelay = true;
        }
        else if (!(gamepad2.right_trigger > 0.5) && !gamepad2.x && rightdelay){
            rightdelay = false;
        }

    }



    private void rightClamp(){
        servo_left.setPosition(0.54); //note: servo_left is left claw, servo_claw is right claw, and servo_right is the rotating servo
        servo_claw.setPosition(0.63);
    }
    private void rightRelease(){
        servo_left.setPosition(0.2);
        servo_claw.setPosition(1.2);
    }
    private void clawHorizontal(){servo_right.setPosition(.34);}
    private void clawVertical(){servo_right.setPosition(0);}
//    private void leftClamp(){ servo_right.setPosition(0.15); }
//    private void leftRelease(){
//        servo_right.setPosition(0.25);
//    }

    private void gamepad22() {
        move_lever_up_and_down();
        move_left_and_right_claw();
        move_linear_slide();
        //  hanging();
    }

    /**
     * Describe this function...
     */
    private void move_linear_slide() {


        switch (slidereststage){
            case correction:
                linear_slide.setPower(gamepad2.right_stick_y*1);
                telemetry.addData("in correction",123);
                telemetry.update();
                if (gamepad2.left_bumper){
                    slidereststage = slidereststages.reset;
                }
                break;

            case reset:
                linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                linear_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addData("in reset",123);
                telemetry.update();


                slidereststage = slidereststages.normal;
                break;
            case normal:
                if (gamepad2.right_bumper){
                   // slideautomoving = true;
                    slidereststage = slidereststages.zero;
                    break;
                }
                double movement = tgtpowerls* gamepad2.right_stick_y+linear_slide.getCurrentPosition();
                if (movement >= slidezero){
                    movement = slidezero;
                }
                else if (movement <= -1970){
                    movement = -1970;
                }
                telemetry.addData("in normal",123);
                pidslide(movement,0.3);
                telemetry.update();
                break;
            case zero:
                RobotLog.ii("robot", "in zero" + slidecontroller.getLastError());
                telemetry.addData("in zero",slidecontroller.getLastError());
                telemetry.update();
//                leftRelease();
                rightRelease();

                pidslide(slidezero,0.3);
                if (slidecontroller.getLastError() <= 3 || (gamepad2.right_stick_y != 0)) {
                    RobotLog.ii("robot", "going to normal" + slidecontroller.getLastError());
                    slidereststage = slidereststages.normal;
                    //slideautomoving = false;
                    break;
                }
                break;
        }
        telemetry.update();

    }

    private void hanging(){
        if (gamepad2.y){
            rigging = true;
        }
    }


    private void move_lever_up_and_down() {
        float tgtPower1;

        tgtPower1 = gamepad2.left_stick_y;
        if (tgtPower1 >0){ //up
            tgtPower1 *= -0.45;
        }
        else{
            tgtPower1 *= -0.3375 ;
        }
        motor_lever.setPower(tgtPower1);
    }
    /**
     * Describe this function...
     */
    private void hold_claw_pos(int lTarget, double power) {
        telemetry.addData("current pos pre", motor_lever.getCurrentPosition());
        telemetry.update();
        motor_lever.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("current pos post", motor_lever.getCurrentPosition());
        telemetry.update();
        motor_lever_pos = motor_lever.getCurrentPosition();
        motor_lever_pos_NEW = lTarget;
        telemetry.addData("motor levere pos new", -1 * motor_lever_pos_NEW);
        if (power < 0) {
            motor_lever.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            motor_lever.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        telemetry.update();
        motor_lever.setTargetPosition(lTarget);
        motor_lever.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_lever.setPower(0.1);
        while (motor_lever.getCurrentPosition() > -1 * motor_lever_pos_NEW) {
            idle();
            telemetry.addData("INLOOP_Original Curr Pos", motor_lever_pos);
            telemetry.addData("In Loop: Curr Pos", motor_lever.getCurrentPosition());
            telemetry.update();
        }
        motor_lever.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_lever.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Motor_lever status", "DONE and stopped");
        telemetry.addData("Original Curr Pos", motor_lever_pos);
        telemetry.addData("Curr Pos", motor_lever.getCurrentPosition());
        telemetry.update();
    }

    /**
     * Describe this function...
     */
//
    private void use_drone_luancher_modified(){
        if (gamepad1.left_bumper) {

            hold_claw_pos(950, 0.2);

            sleep(500);
            servo_drone.setPosition(1.2);
            telemetry.addData("drone", servo_drone.getPosition());
            telemetry.update();
            sleep(1000);

            // sleep(1000);
            servo_drone.setPosition(0);
            motor_lever.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void pidhang(double referacne){ //for PID control
        slidecontroller.setTargetPosition(referacne);
        double correction = slidecontroller.update(linear_slide.getCurrentPosition());

        while (true){
            drive.setMotorPowers(0,0,0,0);
            correction = slidecontroller.update(linear_slide.getCurrentPosition());
            linear_slide.setPower(correction);
        }

    }

    private void pidslide(double referacne,double benchmark){ //for PID control
        slidecontroller.setTargetPosition(referacne);

        double correction = slidecontroller.update(linear_slide.getCurrentPosition());

        if (Math.abs(slidecontroller.getLastError()) != 0){

            correction = slidecontroller.update(linear_slide.getCurrentPosition());

            linear_slide.setPower(correction * (0.5 * limitspeed));

        }

    }


    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public void set_auto_power(){
        double VoltageThresold = 10.3;
        double volt = getBatteryVoltage();
        if (volt <= VoltageThresold){
            limitspeed = 0.75;
        }
        else{
            limitspeed = 1;
        }


    }
}


//UPDATES:
//2/26: added a correction factor to PIDSLIDE in super drive (0.5)