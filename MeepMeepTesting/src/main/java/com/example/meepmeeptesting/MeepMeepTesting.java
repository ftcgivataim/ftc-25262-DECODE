package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    // === Blue Basket Autonomous Parameters ===
    // DECODE Field: 144x144 inches, coordinates from -72 to +72
    // Blue Alliance starts on the left side (negative X)
    // Blue Basket is in the corner at approximately (-60, 60)

    // Starting position - Blue alliance starting zone
    public static double START_X = -12;
    public static double START_Y = -60;
    public static double START_HEADING = Math.toRadians(90);  // Facing towards positive Y

    // Shooting position - center area with clear shot to basket
    public static double SHOOT_X = -36;
    public static double SHOOT_Y = -12;

    // Blue basket is at corner (-72, 72), we aim towards it
    // Angle from shooting position to basket: atan2(72-(-12), -72-(-36)) = atan2(84, -36) ≈ 113°
    public static double BASKET_HEADING = Math.toRadians(115);

    // Blue parking/observation zone (near the submersible)
    public static double PARK_X = -48;
    public static double PARK_Y = -60;
    public static double PARK_HEADING = Math.toRadians(180);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // Blue Basket Autonomous trajectory
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(START_X, START_Y, START_HEADING))
                // Step 1: Drive to shooting position (straight line to avoid going out of bounds)
                .strafeTo(new Vector2d(SHOOT_X, SHOOT_Y))

                // Step 2: Turn to face blue basket
                .turnTo(BASKET_HEADING)

                // Step 3: Wait for shooting (simulated with waitSeconds)
                .waitSeconds(2.5)  // Shooter spin-up + shoot time

                // Step 4: Drive to parking zone (straight movements)
                .strafeTo(new Vector2d(PARK_X, PARK_Y))
                .turnTo(PARK_HEADING)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}