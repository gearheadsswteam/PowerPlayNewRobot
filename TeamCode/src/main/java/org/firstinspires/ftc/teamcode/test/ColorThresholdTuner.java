package org.firstinspires.ftc.teamcode.test;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
public class ColorThresholdTuner extends OpenCvPipeline {
    public Scalar lower = new Scalar(0, 0, 0);
    public Scalar upper = new Scalar(255, 255, 255);
    Mat process = new Mat();
    Mat output = new Mat();
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, process, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(process, lower, upper, process);
        output.release();
        Core.bitwise_and(input, input, output, process);
        return output;
    }
}
