package org.firstinspires.ftc.teamcode.subsystems.centerstage.vision;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PropSensorPipeline extends OpenCvPipeline {
    // left, center, right = 0, 1, 2
    private int propPosition;

    private final double[] upper, lower;
    private static final double[]
            BLUE_UPPER = {150, 255, 60},
            BLUE_LOWER = {0, 85, 0},
            RED_UPPER = {0, 0, 0},
            RED_LOWER = {0, 0, 0};

    public PropSensorPipeline(boolean isBlue) {
        this.upper = isBlue ? BLUE_UPPER : RED_UPPER;
        this.lower = isBlue ? BLUE_LOWER : RED_LOWER;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mask = createMask(input, upper, lower);
        Imgproc.cvtColor(mask, mask, Imgproc.COLOR_BGR2GRAY);
        Rect rect = Imgproc.boundingRect(mask);

        Scalar color = new Scalar(0, 255, 0);
        Point midpoint = new Point(rect.x + (rect.width / 2.0), rect.y + (rect.height / 2.0));

        Imgproc.line(input, new Point(input.width() / 3.0, 0), new Point(input.width() / 3.0, input.height()), color, 5);
        Imgproc.line(input, new Point(2 * input.width() / 3.0, 0), new Point(2 * input.width() / 3.0, input.height()), color, 5);
        Imgproc.rectangle(input, rect, color, 5);
        Imgproc.circle(input, midpoint, 2, color, 10);

        double xPos = midpoint.x / input.cols();
        propPosition = xPos <= 1.0 / 3.0 ? 0 : xPos <= 2.0 / 3.0 ? 1 : 2;

        return input;
    }

    private Mat createMask(Mat input, double[] upper, double[] lower) {
        Mat filtered = new Mat();
        input.copyTo(filtered);

        for(int i = 0; i < input.rows(); i++) {
            for(int j = 0; j < input.cols(); j++) {
                double[] data = input.get(i, j);
                if (data[2] < upper[2] && data[2] > lower[2] && data[1] < upper[1] && data[1] > lower[1] && data[0] < upper[0] && data[0] > lower[0]) {
                    double[] black = {255, 255, 255};
                    filtered.put(i, j, black);
                } else {
                    double[] white = {0, 0, 0};
                    filtered.put(i, j, white);
                }
            }
        }

        return filtered;
    }

    int propPosition() {
        return propPosition;
    }
}
