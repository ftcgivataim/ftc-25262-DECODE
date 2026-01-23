package com.example.meepmeeptesting;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Helpers {
    public static final Pose2D GOAL_POSE_BLUE = new Pose2D(DistanceUnit.INCH, -72, -72, AngleUnit.RADIANS,0);
    public static final Pose2D GOAL_POSE_RED = new Pose2D(DistanceUnit.INCH, -72, 72, AngleUnit.RADIANS,0);

    public static double getAngleToBasket(Pose2D pose, boolean isBlue)  {
        Pose2D goalPose = isBlue ? GOAL_POSE_BLUE : GOAL_POSE_RED;
        return Math.atan2(goalPose.getX(DistanceUnit.INCH) - pose.getX(DistanceUnit.INCH), goalPose.getY(DistanceUnit.INCH) - pose.getY(DistanceUnit.INCH));
    }
}
