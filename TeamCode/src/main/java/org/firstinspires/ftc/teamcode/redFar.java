package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;



@Config

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "drive")
public class redFar extends LinearOpMode {

    //aryan
    private Servo servo_right;
    private Servo servo_left;
    private Servo servo_claw;
    private DcMotor motor_lever;

    private DcMotor linear_slide;
    int motor_lever_pos;
    int motor_lever_pos_NEW;

    Pose2d apend = new Pose2d();

    //for clarity
    private final int RED_alliance = 3;
    private final int BLUE_alliance = 1;
    PIDFController slidecontroller; //for pid control
    PIDCoefficients slidecoeffs; //for pid control
    PIDFController armcontroller; //for pid control
    //
    PIDCoefficients armcoeffs; //for pid control
    @Override
    public void runOpMode() throws InterruptedException {

        // Creating Drivetrain
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        servo_right = hardwareMap.get(Servo.class, "servo_right");
        servo_left = hardwareMap.get(Servo.class, "servo_left");
        servo_claw = hardwareMap.get(Servo.class, "servo_claw");
        motor_lever = hardwareMap.get(DcMotor.class, "motor_lever");
        linear_slide = hardwareMap.get(DcMotor.class, "linear_slide");

        motor_lever.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_lever.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armcoeffs = new PIDCoefficients(-0.025, 0,0); //for pid control
        // create the controller
        armcontroller =  new PIDFController(armcoeffs, 0, 0, 0, (x, v) -> 0.5);

        slidecoeffs = new PIDCoefficients(0.025, 0,0); //for pid control
        slidecontroller = new PIDFController(slidecoeffs); //for pid control

        Pose2d startPose = new Pose2d(0, 0, 0); // starting position is 0,0,0
        drive.setPoseEstimate(startPose);


        //right
        TrajectorySequence main = drive.trajectorySequenceBuilder(startPose)
                // Move forward
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    hold_claw_pos(400,0.75);
                    sleep(10);
                })
                .waitSeconds(0.1)

                .lineToLinearHeading(new Pose2d(24.25, 4, 0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    hold_claw_pos(1000,0.25);
                    sleep(10);
                    slide(1400,0.5);
                    sleep(10);
                })
                .waitSeconds(0.1)

                .lineToLinearHeading((new Pose2d(29.75, 4,0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    slide(1000,-0.5);
                    sleep(10);
                    rightRelease();
                    sleep(100);
                    slide(150,-0.5);
                    sleep(10);
                })
                .waitSeconds(0.5)

                .strafeTo(new Vector2d(27,-33))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    hold_claw_pos(1400,-0.5);
                    sleep(10);
                    rightClamp();
                    sleep(500);
                })
                .waitSeconds(0.5)

                .lineToLinearHeading(new Pose2d(13,-33,3.14))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    rightRelease();
                    sleep(100);
                })
                .waitSeconds(0.5)

                .lineToLinearHeading(new Pose2d(27,-47,0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    rightClamp();
                    sleep(500);
                })
                .waitSeconds(0.5)

                .lineToLinearHeading(new Pose2d(15,-47,3.14))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    rightRelease();
                    sleep(100);
                })
                .waitSeconds(0.5)

                .strafeTo(new Vector2d(13,-33))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    rightClamp();
                    sleep(500);
                    hold_claw_pos(1400, 0.5);
                })
                .waitSeconds(0.5)

                .lineToLinearHeading(new Pose2d(24.25, 12, 0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    slide(1400,0.5);
                    sleep(10);
                })
                .waitSeconds(0.1)

                .lineToLinearHeading((new Pose2d(30, 12,0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    slide(1000,-0.5);
                    sleep(10);
                    rightRelease();
                    sleep(10);
                    slide(150,-0.5);
                    sleep(10);
                })
                .waitSeconds(0.1)

                .lineToLinearHeading(new Pose2d(10,-33))


//
//                .lineToLinearHeading(new Pose2d(10.5, 43.5, 2.35))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    rightRelease();
//                    sleep(100);
//                })
//                .waitSeconds(1.0)
//
//                .lineToLinearHeading(new Pose2d(28,41,0))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    slide(1400,-0.5);
//                    sleep(100);
//                    hold_claw_pos(1400,-0.5);
//                    sleep(100);
//                    rightClamp();
//                    sleep(500);
//                    hold_claw_pos(1400,0.5);
//                })
//                .waitSeconds(1.0)
////
//                .lineToLinearHeading(new Pose2d(14,36.5,2.36))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    slide(1600, 0.5);
//                    sleep(100);
//                })
//                .waitSeconds(1.0)
////
//                .lineToLinearHeading(new Pose2d(10.5, 43.5, 2.36))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    rightRelease();
//                    sleep(100);
//                })
//                .waitSeconds(1.0)


                .build();

        // leftClamp();
        rightClamp();
        clawHorizontal();


        waitForStart();

        drive.followTrajectorySequence(main);

    }



    private void slide(int LStarget, double LSpower) {
        linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("current pos post", linear_slide.getCurrentPosition());
        telemetry.update();
        motor_lever_pos = linear_slide.getCurrentPosition();
        motor_lever_pos_NEW = LStarget;
        telemetry.addData("motor levere pos new", motor_lever_pos_NEW);
        telemetry.update();
        if (LSpower < 0) {
            linear_slide.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            linear_slide.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        linear_slide.setTargetPosition(LStarget);
        linear_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear_slide.setPower(LSpower);
        while (linear_slide.getCurrentPosition() < motor_lever_pos_NEW) {
            idle();
            telemetry.addData("INLOOP_Original Curr Pos", motor_lever_pos);
            telemetry.addData("In Loop: Curr Pos", linear_slide.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addData("Motor_lever status", "DONE and stopped");
        telemetry.update();
        linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Motor_lever status", "DONE and stopped");
        telemetry.addData("Original Curr Pos", motor_lever_pos);
        telemetry.addData("Curr Pos", linear_slide.getCurrentPosition());
        telemetry.update();
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

    private void hold_claw_pos(int lTarget, double power) {
        telemetry.addData("current pos pre", motor_lever.getCurrentPosition());
        telemetry.update();
        motor_lever.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("current pos post", motor_lever.getCurrentPosition());
        telemetry.update();
        motor_lever_pos = motor_lever.getCurrentPosition();
        motor_lever_pos_NEW = lTarget;
        telemetry.addData("motor levere pos new", -1 * motor_lever_pos_NEW);
        telemetry.update();
        if (power < 0) {
            motor_lever.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            motor_lever.setDirection(DcMotorSimple.Direction.FORWARD);
        }
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

    private void pidslide(double reference) { //for PID control
        slidecontroller.setTargetPosition(reference);
        double correction = slidecontroller.update(linear_slide.getCurrentPosition()); //returns power
        while ((Math.abs(slidecontroller.getLastError()) > 10)) { // allowed error was set to 0.2; relaxed to 10
            correction = slidecontroller.update(linear_slide.getCurrentPosition());
//            if(correction > 1) {
//                correction = 1;
//            }
//            else {
//                correction = correction * 1.20;
//            }
            linear_slide.setPower(correction);
        }
    }

    public void pidcontrol(PIDFController controller, DcMotor motor, double targetpos , double speed, Telemetry t  ) {
        controller.setTargetPosition(targetpos);
        double correction; //returns power
        boolean skip = true;

        while ((controller.getLastError() > 10) || (skip)) {
            skip = false;
            correction = controller.update(motor.getCurrentPosition());

            t.addData("currentpos", motor.getCurrentPosition());
            t.addData("referance", controller.getTargetPosition());
            t.addData("correc", correction);
            t.addData("slide", motor.getPower());
            t.update();
            motor.setPower(correction *speed);
        }

    }
}