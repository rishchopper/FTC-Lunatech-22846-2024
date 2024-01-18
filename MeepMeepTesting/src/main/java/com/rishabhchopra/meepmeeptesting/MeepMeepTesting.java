package com.rishabhchopra.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.65, -62.4, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(11.65, -35, Math.toRadians(180)))
                                .lineTo(new Vector2d(0, -35))
                                .splineToConstantHeading(new Vector2d(-23, -60), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-60, -35), Math.toRadians(180))
                                .strafeTo(new Vector2d(-55, -12))
                                .lineTo(new Vector2d(13, -11))
                                .lineTo(new Vector2d(35, -19.2))
                                .lineTo(new Vector2d(50, -35))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}