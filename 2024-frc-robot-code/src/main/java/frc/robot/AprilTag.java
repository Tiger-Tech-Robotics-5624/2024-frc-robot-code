// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;

import edu.wpi.first.apriltag.AprilTagDetector;

public class AprilTag {
  private Point pt0;
  private Point pt1;
  private Point pt2;
  private Point pt3;
  private Point center;

  private Mat grayMat;

  private AprilTagDetector detector; 
  
  public AprilTag() {
    grayMat = new Mat();
    detector = new AprilTagDetector();

    var results = detector.detect(grayMat);

    for (var result : results) {
      
    }
  }
}
