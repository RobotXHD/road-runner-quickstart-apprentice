package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class OpenCVTest extends LinearOpMode {
    private OpenCvCamera webcam;
    private ContourPipeline pipeline;

    int varrez;
    private double crThreshHigh = 150;
    private double crThreshLow = 120;
    private double cbThreshHigh = 255;
    private double cbThreshLow = 255;

    private int minRectangleArea = 2000;
    private double leftBarcodeRangeBoundary = 0.3; //i.e 30% of the way across the frame from the left
    private double rightBarcodeRangeBoundary = 0.6; //i.e 60% of the way across the frame from the left

    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    @Override
    public void runOpMode() throws InterruptedException {//mai merge teamviewedraaaaa-aauaalaaaaaa da merge
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new ContourPipeline(0, 0, 0, 0);

        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        //cred ca merge in loc de wait for start
        while(!isStopRequested()){
            //webcamul se porneste async (nu blocheza programul pana se porneste
            //deci trebuie asteptat cu detectia pana cand e pornit
            //ok
            //testeaza acum si sa veden cum merge si ce erori da
            //
            try{
                double rectangleArea = pipeline.getRectArea();

                //Print out the area of the rectangle that is found.
                telemetry.addData("Rectangle Area", rectangleArea);
                telemetry.addData("Rectangle Height:", pipeline.getRectHeight());
                telemetry.addData("Rectangle Width:", pipeline.getRectWidth());
                telemetry.addData("Position","X = " + pipeline.getRectX() + "    Y = " + pipeline.getRectY());

                //Check to see if the rectangle has a large enough area to be a marker.
                if (rectangleArea > minRectangleArea) {
                    //Then check the location of the rectangle to see which barcode it is in.
                    if (pipeline.getRectMidpointX() > rightBarcodeRangeBoundary * pipeline.getRectWidth()){
                        telemetry.addData("Barcode Position", "Right");
                    } else if (pipeline.getRectMidpointX() < leftBarcodeRangeBoundary * pipeline.getRectWidth()) {
                        telemetry.addData("Barcode Position", "Left");
                    } else {
                        telemetry.addData("Barcode Position", "Center");
                    }
                }
                telemetry.update();
            }
            catch (Exception e){
                telemetry.addData("E: ", e.getMessage());
                telemetry.update();
                //:)
            }
            while(!isStopRequested()){

            }
        }
    }
    public double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }
}
