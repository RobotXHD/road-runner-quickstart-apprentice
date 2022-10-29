package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var.Webcam_h;
import static org.firstinspires.ftc.teamcode.Var.Webcam_w;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Var.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class OpenCV_PID extends LinearOpMode {
    private Servo servo;
    private OpenCvWebcam webcam;;
    private PachetelNouOpenCV pipeline = new PachetelNouOpenCV();
    private double correction, servoPosition = 0.5;
    private PidControllerAdevarat pid = new PidControllerAdevarat(kp,ki,kd);
    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class,"servo");
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(Webcam_w, Webcam_h, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        telemetry.addLine("Waiting for start");
        telemetry.update();
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        pid.setSetpoint(Webcam_w /2);
        pid.enable();

        waitForStart();
        while (opModeIsActive()) {
            pid.setPID(kp,ki,kd);
            correction = pid.performPID(pipeline.getRect().x+pipeline.getRect().width/2);
            if(servoPosition - correction < 0){
                servoPosition = 0;
            }
            else if(servoPosition - correction > 1){
                servoPosition = 1;
            }
            else{
                servoPosition -= correction;
            }
            servo.setPosition(servoPosition);
            telemetry.addData("setPoint",pid.getSetpoint());
            telemetry.addData("input",pipeline.getRect().x+pipeline.getRect().width/2);
            telemetry.addData("P",pid.getP());
            telemetry.addData("I",pid.getI());
            telemetry.addData("D",pid.getD());
            telemetry.addData("correction",correction);
            telemetry.update();
        }
    }
}
