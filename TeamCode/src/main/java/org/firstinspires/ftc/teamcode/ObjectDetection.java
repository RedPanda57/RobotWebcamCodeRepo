/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Object Detection", group="Robot")
public class ObjectDetection extends LinearOpMode
{
    OpenCvWebcam webcam;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        GreenSphereDetectionPipeline colorDetectionPipeline = new GreenSphereDetectionPipeline(webcam);
        webcam.setPipeline(colorDetectionPipeline);
        webcam.setMillisecondsPermissionTimeout(5000);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Camera Status", "Stream gestart");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Camera Fout", "Code: " + errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());

            telemetry.addData("Afstand tot bal (cm)", String.format("%.2f", colorDetectionPipeline.getLastDistance()));
            telemetry.addData("Aantal groene objecten", colorDetectionPipeline.getNumberOfGreenObjects());

            // Positiemelding van de bal
            String positie = "Geen bal gedetecteerd";
            if (colorDetectionPipeline.getNumberOfGreenObjects() > 0) {
                if (colorDetectionPipeline.isBallCentered()) {
                    positie = "BAL IN MIDDEN";
                } else if (colorDetectionPipeline.getLastCenter().x < 160) {
                    positie = "BAL LINKS";
                } else {
                    positie = "BAL RECHTS";
                }
            }
            telemetry.addData("Positie van bal", positie);
            telemetry.update();

            if(gamepad1.a)
            {
                webcam.stopStreaming();
            }

            sleep(100);
        }

        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    class GreenSphereDetectionPipeline extends OpenCvPipeline
    {
        boolean viewPortPaused;

        private static final int CAMERA_WIDTH = 320;
        private static final int CAMERA_HEIGHT = 240;
        private static final int CENTER_TOLERANCE = 20;

        Mat hsv = new Mat();
        Mat mask = new Mat();
        Mat hierarchy = new Mat();

        Scalar lowerGreen = new Scalar(35, 100, 40);
        Scalar upperGreen = new Scalar(85, 255, 255);

        private volatile int numberOfGreenObjects = 0;
        private volatile double lastRadius = 0;
        private volatile double lastDistance = 0;
        private volatile Point lastCenter = new Point(-1, -1);
        private volatile boolean isCentered = false;

        private OpenCvWebcam externalWebcam;

        public GreenSphereDetectionPipeline(OpenCvWebcam webcam)
        {
            this.externalWebcam = webcam;
        }

        public int getNumberOfGreenObjects() { return numberOfGreenObjects; }
        public double getLastRadius() { return lastRadius; }
        public double getLastDistance() { return lastDistance; }
        public boolean isBallCentered() { return isCentered; }
        public Point getLastCenter() { return lastCenter; }

        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsv, lowerGreen, upperGreen, mask);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            numberOfGreenObjects = 0;
            double maxRadius = 0;
            Point bestCenter = new Point(-1, -1);

            for (MatOfPoint contour : contours)
            {
                double contourArea = Imgproc.contourArea(contour);

                if (contourArea > 500)
                {
                    Point center = new Point();
                    float[] radius = new float[1];
                    MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                    Imgproc.minEnclosingCircle(contour2f, center, radius);

                    double pixelRadius = radius[0];
                    double pixelDiameter = 2 * pixelRadius;
//dddfdfdddd
                    double knownDistance = 30.0;
                    double realDiameter = 12.70;
                    double measuredPixelDiameter = 0; // Moet je nog kalibreren!
                    double focalLength = (measuredPixelDiameter * knownDistance) / realDiameter;
                    double distance = (realDiameter * focalLength) / pixelDiameter;

                    if (pixelRadius > maxRadius) {
                        maxRadius = pixelRadius;
                        lastDistance = distance;
                        lastRadius = pixelRadius;
                        bestCenter = center;
                    }

                    Imgproc.circle(input, center, (int) pixelRadius, new Scalar(0, 255, 0), 2);
                    Imgproc.circle(input, center, 3, new Scalar(0, 0, 255), -1);

                    numberOfGreenObjects++;
                    contour2f.release();
                }
                contour.release();
            }

            lastCenter = bestCenter;

            if (bestCenter.x > 0 && bestCenter.y > 0) {
                double centerX = CAMERA_WIDTH / 2.0;
                double centerY = CAMERA_HEIGHT / 2.0;

                isCentered = Math.abs(bestCenter.x - centerX) <= CENTER_TOLERANCE &&
                        Math.abs(bestCenter.y - centerY) <= CENTER_TOLERANCE;

                Imgproc.rectangle(input,
                        new Point(centerX - CENTER_TOLERANCE, centerY - CENTER_TOLERANCE),
                        new Point(centerX + CENTER_TOLERANCE, centerY + CENTER_TOLERANCE),
                        new Scalar(255, 0, 0), 2);
            } else {
                isCentered = false;
            }

            hsv.release();
            mask.release();
            hierarchy.release();

            return input;
        }

        @Override
        public void onViewportTapped()
        {
            viewPortPaused = !viewPortPaused;
            if(viewPortPaused)
            {
                externalWebcam.pauseViewport();
            }
            else
            {
                externalWebcam.resumeViewport();
            }
        }
    }
}
