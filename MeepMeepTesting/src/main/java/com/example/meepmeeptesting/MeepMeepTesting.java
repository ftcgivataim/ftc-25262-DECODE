package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.CompositeVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> 10.0;

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(63, -9, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-18,-18), Math.atan2(-72+18,18-72)+Math.PI)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(36,-24), Math.PI*3/2)
                .waitSeconds(1)
                .strafeTo(new Vector2d(36,-56),baseVelConstraint)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(36,-24), Math.PI*3/2)
                .strafeToLinearHeading(new Vector2d(-18,-18), Math.atan2(-72+18,18-72)+Math.PI)
                .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(36,-32),0)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}