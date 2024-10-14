package org.firstinspires.ftc.teamcode;

import static java.lang.String.format;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;


@Config

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "drive")
public class rednearv2 extends LinearOpMode {


    public enum actions {
        EMPTY,
        INIT,
        CLAWOPEN,
        CLAWCLOSE,
        ROTATE90,
        WAIT,

        WAITTILL
    }

    //aryan
    private Servo servo_left;
    private Servo servo_rotate;

    int targetclaw;
    int targetslide;


    PIDFController slidecontroller; //for pid control
    PIDFController armcontroller; //for pid control
    CPID slide;
    CPID arm;


    double waititme;


    @Override
    public void runOpMode() throws InterruptedException {

        // Creating Drivetrain
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        final actions[] currentaction = {actions.INIT, actions.EMPTY};

        final double[] waitill = {0, 0};


       // servo_rotate = hardwareMap.get(Servo.class, "servo_rotate");
        servo_left = hardwareMap.get(Servo.class, "servo_left");

        // create the controller
        armcontroller = new PIDFController(new PIDCoefficients(0.025, 0.00001, 0.0001));
        arm = new CPID();
        arm.init(hardwareMap.get(DcMotor.class, "motor_lever"), armcontroller, -1);

        slidecontroller = new PIDFController(new PIDCoefficients(0.025, 0, 0)); //for pid control
        slide = new CPID();
        slide.init(hardwareMap.get(DcMotor.class, "linear_slide"), slidecontroller, 0.5);

        ElapsedTime timer = new ElapsedTime();


        Pose2d startPose = new Pose2d(0, 0, 0); // starting position is 0,0,0
        drive.setPoseEstimate(startPose);

        TrajectorySequence main = drive.trajectorySequenceBuilder(startPose)
                // Move forward

                .addDisplacementMarker(() -> {
                    targetclaw = -1000;
                   // targetslide = 1500;

                })

                .lineToLinearHeading(new Pose2d(17, -4, 0))

                .lineToLinearHeading((new Pose2d(21, -4, 0)))

                .addDisplacementMarker(() -> {
                  //  targetslide = 500; //1500-1000
                    currentaction[0] = actions.CLAWOPEN;
                })

//                .strafeTo(new Vector2d(17,31.5))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
//                    targetclaw = 200;
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
//                    currentaction[0] = actions.CLAWOPEN;
//                    currentaction[1] = actions.WAIT;
//                    waititme = 0.3;
//                    targetclaw = 1000;
//                })
//
//                .lineToLinearHeading(new Pose2d(6,36.5,2.36))
//
//                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> {
//                    targetslide = 1500;
//                })
//
//                .lineToLinearHeading(new Pose2d(2,39.5,2.36))
//
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    currentaction[0] = actions.CLAWOPEN;
//                    currentaction[1] = actions.WAIT;
//                    waititme = 0.1;
//                })


                .build();

        rightClamp();
        targetslide = 0;
        targetclaw = 0;

        waitForStart();

        drive.followTrajectorySequenceAsync(main);

        while (opModeIsActive() && !isStopRequested()) {
            timer.reset();

            telemetry.addData("slidetarget", targetslide);
            telemetry.addData("currentaction", currentaction[0]);



            switch (currentaction[0]) { //this is for attahcmetns and waits so far
                case INIT:
                    //nothing just here
                    telemetry.addLine("init");
                case CLAWOPEN:
                    rightRelease();
                    currentaction[0] = currentaction[1];
                    currentaction[1] = actions.EMPTY;
                    break;

                case CLAWCLOSE:
                    rightClamp();
                    currentaction[0] = currentaction[1];
                    currentaction[1] = actions.EMPTY;
                    break;
                case ROTATE90:
                    servo_rotate.setPosition(90); //some value that turns it 90 degrees all the time
                    currentaction[0] = currentaction[1];
                    currentaction[1] = actions.EMPTY;
                    break;

                case WAIT:

                    if (timer.milliseconds() > waititme) {  // 500 ms delay
                        currentaction[0] = currentaction[1];  // Move to the next action after waiting
                        currentaction[1] = actions.EMPTY;  // Reset second action
                    }
                    currentaction[0] = currentaction[1];
                    currentaction[1] = actions.EMPTY;
                    break;

                case WAITTILL:
                    if (waitill[0] == 1) {
                        if (waitill[1] == slide.getmotor().getCurrentPosition()) {
                            currentaction[0] = currentaction[1];
                            currentaction[1] = actions.EMPTY;
                        }
                    }

                    if (waitill[0] == 2) {
                        if (waitill[1] == arm.getmotor().getCurrentPosition()) {
                            currentaction[0] = currentaction[1];
                            currentaction[1] = actions.EMPTY;
                        }
                    }
                    break;
            }//end of attachemtn state machiene
            if (!(currentaction[0] == actions.WAIT)) {

                telemetry.addLine(format("arm %s", arm.update(targetclaw)));
                telemetry.addData("time it takes",timer.time());
                telemetry.update();
                drive.update();
            }
            telemetry.update();

        }
    }

    private void rightClamp () {
        servo_left.setPosition(0.44);
    }
    private void rightRelease () {
        servo_left.setPosition(0.2);
    }

    class CPID {

        private double target;
        private double correction;
        private DcMotor selfmotor;
        private PIDFController selfcontroller;

        private  double selfpower;

        public void init(DcMotor motor, PIDFController controller, double power) {
            this.selfmotor = motor;
            this.target = 0;
            this.correction = 0;
            this.selfcontroller = controller;
            this.selfpower = power;

            this.selfmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.selfmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.selfmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        }

        public String update(double t) {
            if (!(this.target == t)){
                this.target = t;
                this.selfcontroller.setTargetPosition(this.target);
            }


            this.correction = selfcontroller.update(selfmotor.getCurrentPosition());

            this.selfmotor.setPower(-1 *Math.max(-1,Math.min(1,this.correction)));

            return String.valueOf(this.correction*this.selfpower);




        }

        public DcMotor getmotor() {
            return this.selfmotor;
        }

    }//cpid class

}// whole thing class
