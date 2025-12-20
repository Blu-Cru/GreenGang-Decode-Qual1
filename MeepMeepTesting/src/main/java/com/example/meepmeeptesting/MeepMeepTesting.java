package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    private static double deg(double d) {
        return Math.toRadians(d);
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(69, 69, deg(180), deg(180), 17)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(new Pose2d(-46.5, -51.5, deg(234.0949)))
                        .waitSeconds(5)
                        .setReversed(true)
                        .setTangent(deg(-45))
                        .splineToSplineHeading(
                                new Pose2d(-24, 24, deg(-225)),
                                deg(-45)
                        )

                        // .waitSeconds(1)

                        // shoot

                        .setReversed(false)
                        .setTangent(deg(0))
                        .splineToSplineHeading(
                                new Pose2d(-12, 30, deg(90)),
                                deg(90)
                        )

                        // .waitSeconds(1)

                        // intake
                        .lineToY(50)

                        .setReversed(true)
                        .setTangent(deg(-112))
                        .splineToSplineHeading(
                                new Pose2d(-24, 24, deg(-225)),
                                deg(-112)
                        )
                        //.waitSeconds(1)

                        // shoot

                        .setReversed(false)
                        .setTangent(deg(0))
                        .splineToSplineHeading(
                                new Pose2d(14, 35, deg(90)),
                                deg(90)
                        )

                        // .waitSeconds(1)

                        // intake
                        .lineToY(55)

                        .setReversed(true)
                        .setTangent(deg(-60))
                        .splineToSplineHeading(
                                new Pose2d(-24, 24, deg(-225)),
                                deg(-139)
                        )

                        //.waitSeconds(1)

                        .setReversed(false)
                        .setTangent(deg(0))
                        .splineToSplineHeading(
                                new Pose2d(36, 40, deg(90)),
                                deg(90)
                        )

                        //.waitSeconds(1)

                        .lineToY(55)

                        //.waitSeconds(1)

                        .setReversed(true)
                        .setTangent(deg(-130))
                        .splineToSplineHeading(
                                new Pose2d(-24, 24, deg(-225)),
                                deg(-153)
                        )
                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
