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
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(69, 69, deg(180), deg(180), 17)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(new Pose2d(-48, -48, deg(225)))

                        .setReversed(true)
                        .setTangent(deg(45))
                        .splineToSplineHeading(
                                new Pose2d(-24, -24, deg(225)),
                                deg(45)
                        )

                        // shoot

                        .setReversed(false)
                        .setTangent(deg(0))
                        .splineToSplineHeading(
                                new Pose2d(-12, -22, deg(-90)),
                                deg(-90)
                        )

                        // intake
                        .lineToY(-50)

                        .setReversed(true)
                        .setTangent(deg(112))
                        .splineToSplineHeading(
                                new Pose2d(-24, -24, deg(225)),
                                deg(112)
                        )

                        // shoot

                        .setReversed(false)
                        .setTangent(deg(0))
                        .splineToSplineHeading(
                                new Pose2d(14, -22, deg(-90)),
                                deg(-90)
                        )

                        // intake
                        .lineToY(-55)

                        .setReversed(true)
                        .setTangent(deg(60))
                        .splineToSplineHeading(
                                new Pose2d(-24, -24, deg(225)),
                                deg(139)
                        )


                        .setReversed(false)
                        .setTangent(deg(0))
                        .splineToSplineHeading(
                                new Pose2d(36, -22, deg(-90)),
                                deg(-90)
                        )

                        .lineToY(-55)

                        .setReversed(true)
                        .setTangent(deg(130))
                        .splineToSplineHeading(
                                new Pose2d(-24, -24, deg(225)),
                                deg(153)
                        )
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
