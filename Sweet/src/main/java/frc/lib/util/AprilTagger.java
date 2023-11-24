// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class AprilTagger {

    public AprilTagger() {

        var visionThread = new Thread(() -> apriltagVisionThreadproc());
        visionThread.setDaemon(true);
        visionThread.start();
    }

    public void apriltagVisionThreadproc() {
        var detector = new AprilTagDetector();

        // look for tag16h5, don't correct any error bits
        detector.addFamily("tag16h5", 0);

        // Set up Pose Estimator
        var poseEstimateConfig = new AprilTagPoseEstimator.Config(0, 0, 0, 0, 0);
        var estimator = new AprilTagPoseEstimator(poseEstimateConfig);

        // USB Camera
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(640, 480);

        // Get a CvSink. This will capture Mats from the camera
        CvSink cvSink = CameraServer.getVideo();
        // Setup a CvSource. This will send images back to the Dashboard
        CvSource outputStream = CameraServer.putVideo("Detected", 640, 480);

        // Mats are very memory expensive. Lets reuse these.
        var mat = new Mat();
        var grayMat = new Mat();

        // Instantiate once
        ArrayList<Long> tags = new ArrayList<>();
        var outlineColor = new Scalar(0, 255, 0);
        var crossColor = new Scalar(0, 0, 255);

        // Output to NT
        NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("apriltags");
        IntegerArrayPublisher pubTags = tagsTable.getIntegerArrayTopic("tags").publish();

        // This cannot be true. The program will never exit if it is.
        // This lets the robot stop this thread when restarting robot code or deploying.
        while(!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it in the source mat.
            if (cvSink.grabFrame(mat) == 0) {
                // Send the output the error
                outputStream.notifyError(cvSink.getError());
                // skip the rest of the current iteration
                continue;
            }

            Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_BGR2GRAY);

            AprilTagDetection[] detections = detector.detect(grayMat);

            // have not seen any tags yet
            tags.clear();

            for (AprilTagDetection detection : detections) {
                // remeber that we saw this tag
                tags.add((long) detection.getId());

                // draw lines around the tag
                for (var i = 0; i <= 3; i++) {
                    var j = (i +1) % 4;
                    var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
                    var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
                    Imgproc.line(mat, pt1, pt2, outlineColor, 2);
                }

                // mark the center of the tag
                var cx = detection.getCenterX();
                var cy = detection.getCenterY();
                var ll = 10;
                Imgproc.line(mat, new  Point(cx - ll, cy), new Point(cx + ll, cy), crossColor, 2);
                Imgproc.line(mat, new  Point(cx, cy - ll), new Point(cx, cy + ll), crossColor, 2);

                // identify the tag
                Imgproc.putText(mat, Integer.toString(detection.getId()), new Point(cx + ll, cy), Imgproc.FONT_HERSHEY_SIMPLEX, 1, crossColor, 3);

                // determine pose
                Transform3d pose = estimator.estimate(detection);

                // put pose into dashboard
                Rotation3d rot = pose.getRotation();
                tagsTable.getEntry("pose_" + detection.getId()).setDoubleArray(new double[] {
                    pose.getX(), pose.getY(), pose.getZ(), rot.getX(), rot.getY(), rot.getZ()
                });
            }

            // put list of tags onto dashboard
            pubTags.set(tags.stream().mapToLong(Long::longValue).toArray());

            // give the output stream a new image to display
            outputStream.putFrame(mat);
        }

        pubTags.close();
        detector.close();
    }
}