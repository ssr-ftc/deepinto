package org.firstinspires.ftc.teamcode.roadrunner.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class ZoneChooser extends OpenCvPipeline {
    private OpenCvCamera cam;
    private Mat mat = new Mat();
    private Rect upperROI = new Rect(new Point(240,120),new Point(304,145));
    private Rect lowerROI = new Rect(new Point(240,145),new Point(304,178));
    private Mat upperMat;
    private Mat lowerMat;
    private Target target;
    private Telemetry telemetry;

    public ZoneChooser(HardwareMap hwMap, Telemetry t) {
        telemetry = t;
        //initialize camara can be from one of the same code from EasyOpenCV
        int camMonViewId = hwMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hwMap.appContext.getPackageName()
        );
        cam = OpenCvCameraFactory.getInstance().createWebcam(
                hwMap.get(WebcamName.class, "Webcam"),
                camMonViewId
        );
        cam.setPipeline(this);
//        cam.openCameraDeviceAsync(
//                cam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT)
//
       //);

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            public void onOpened() {
                cam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {

            }

        }) ;


    }
    @Override
    public Mat processFrame(Mat input) {
        //----THRESHOLDIN -------------------------------------------------------------------
//        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2BGR); //  this required for Logitec C920S
//        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2BGR); //  this required for Logitec C920S
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2HSV);
        Scalar lowerBound = new Scalar(15.0/2, 100, 100);
        Scalar upperBound = new Scalar(45.0/2, 100, 100);
        Core.inRange(mat, lowerBound, upperBound, mat);         // ----This function actually performs the thresholding
        //----DIVIDE SCREEN -------------------------------------------------------------------
        upperMat = mat.submat(upperROI);
        lowerMat = mat.submat(lowerROI);
        //----AVERAGE -------------------------------------------------------------------
        double uppervalue = Math.round(Core.mean(upperMat).val[2] / 255);
        double lowervalue = Math.round(Core.mean(lowerMat).val[2] / 255);
        upperMat.release();
        lowerMat.release();
        mat.release();
        //----COMPARE -------------------------------------------------------------------
        final double THRESHOLD = 10; // we will need to find what this needs to be
        if(uppervalue > THRESHOLD) {
            target = Target.A;
        } else if (lowervalue > THRESHOLD) {
            target = Target.B;
        } else {
            target = Target.C;
        }
        return null;
    }

    public Target getTarget() {
        return target;
    }

    public void stop() {
        cam.closeCameraDeviceAsync(() -> {});
    }
}
