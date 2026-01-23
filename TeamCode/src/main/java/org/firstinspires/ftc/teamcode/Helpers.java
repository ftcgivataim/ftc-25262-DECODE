package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

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

    public static Pose2D pose2dToPose2D(Pose2d pose2d) {
        return new Pose2D(DistanceUnit.INCH, pose2d.component1().x, pose2d.component1().y, AngleUnit.RADIANS, pose2d.component2().real);

        }


    public static Pose2d pose2DToPose2d(Pose2D pose2D){
        return new Pose2d(pose2D.getX(DistanceUnit.INCH), pose2D.getY(DistanceUnit.INCH), pose2D.getHeading(AngleUnit.RADIANS));
    }
}
