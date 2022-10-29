package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var.Day_Hhigh;
import static org.firstinspires.ftc.teamcode.Var.Day_Hlow;
import static org.firstinspires.ftc.teamcode.Var.Night_Hhigh;
import static org.firstinspires.ftc.teamcode.Var.Night_Hlow;
import static org.firstinspires.ftc.teamcode.Var.Day_Shigh;
import static org.firstinspires.ftc.teamcode.Var.Day_Slow;
import static org.firstinspires.ftc.teamcode.Var.Night_Shigh;
import static org.firstinspires.ftc.teamcode.Var.Night_Slow;
import static org.firstinspires.ftc.teamcode.Var.Day_Vhigh;
import static org.firstinspires.ftc.teamcode.Var.Day_Vlow;
import static org.firstinspires.ftc.teamcode.Var.Night_Vhigh;
import static org.firstinspires.ftc.teamcode.Var.Night_Vlow;
import static org.firstinspires.ftc.teamcode.Var.CV_detectionType;
import static org.firstinspires.ftc.teamcode.Var.CV_kernel_pult_size;
import static org.firstinspires.ftc.teamcode.Var.CV_rect_x1;
import static org.firstinspires.ftc.teamcode.Var.CV_rect_x2;
import static org.firstinspires.ftc.teamcode.Var.CV_rect_y1;
import static org.firstinspires.ftc.teamcode.Var.CV_rect_y2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class PachetelNouOpenCV extends OpenCvPipeline {
    private final int elementType = Imgproc.CV_SHAPE_RECT;
    private Rect dreptunghi;
    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    @Override
    public Mat processFrame(Mat input) {
        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */


        Mat element = Imgproc.getStructuringElement(elementType, new Size(2 * CV_kernel_pult_size + 1, 2 * CV_kernel_pult_size + 1),
                new Point(CV_kernel_pult_size, CV_kernel_pult_size));
        Mat original = input.clone();

        Scalar scalarLowerHSV,scalarUpperHSV;

        if(CV_detectionType == Var.DetectionTypes.DAY) {
            scalarLowerHSV = new Scalar(Day_Hlow, Day_Slow, Day_Vlow);
            scalarUpperHSV = new Scalar(Day_Hhigh, Day_Shigh, Day_Vhigh);
        }
        else{
            scalarLowerHSV = new Scalar(Night_Hlow, Night_Slow, Night_Vlow);
            scalarUpperHSV = new Scalar(Night_Hhigh, Night_Shigh, Night_Vhigh);
        }

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
        Core.inRange(input, scalarLowerHSV, scalarUpperHSV, input);

        Imgproc.erode(input, input, element);
        Imgproc.dilate(input, input, element);
        Imgproc.dilate(input, input, element);
        Imgproc.erode(input, input, element);

        Mat rect = new Mat(input.rows(), input.cols(), input.type(), Scalar.all(0));
        Imgproc.rectangle(
                rect,
                new Point(CV_rect_x1, CV_rect_y1),
                new Point(CV_rect_x2, CV_rect_y2),
                new Scalar(255),
                Imgproc.FILLED
        );
        Core.bitwise_and(input,rect,input);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(input, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Collections.sort(contours, new Comparator<MatOfPoint>() {
            @Override
            public int compare(MatOfPoint matOfPoint, MatOfPoint t1) {
                return (int)(Imgproc.contourArea(t1) - Imgproc.contourArea(matOfPoint));
            }
        });
        if(!contours.isEmpty()) {
            setRect(Imgproc.boundingRect(contours.get(0)));
        }

        Imgproc.cvtColor(input, input, Imgproc.COLOR_GRAY2RGBA);
        Core.bitwise_or(input, original, input);

        Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 0), 4);

        Imgproc.rectangle(
                input,
                new Point(CV_rect_x1, CV_rect_y1),
                new Point(CV_rect_x2, CV_rect_y2),
                new Scalar(255, 127, 0), 4);

        Imgproc.rectangle(
                input,
                getRect(),
                new Scalar(0, 255, 255), 4);

        original.release();
        rect.release();

        return input;
    }

    public void setRect(Rect rect) {
        this.dreptunghi = rect;
    }

    public Rect getRect() {
        return dreptunghi;
    }
}
