package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(69, 69, Math.toRadians(180), Math.toRadians(180), 17)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(new Pose2d(63, -24, Math.toRadians(180)))

                        .setTangent(Math.toRadians(160))
                        .splineToSplineHeading(
                                new Pose2d(-24, -24, Math.toRadians(225)),
                                Math.toRadians(180)
                        )

                        //shoot

                        // Hub -> -12,-22: end facing -90, approach downward so lineToY is clean
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(
                                new Pose2d(-12, -22, Math.toRadians(-90)),
                                Math.toRadians(-90)
                        )

                        //intake
                        .lineToY(-55 )

                        .setReversed(true)

                        // Return -> Hub: keep facing 225, but arc UP into hub (end tangent ~135)
                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(
                                new Pose2d(-24, -24, Math.toRadians(225)),
                                Math.toRadians(135)
                        )

                        //shoot


                        // Hub -> +12,-22: same idea
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(
                                new Pose2d(12, -22, Math.toRadians(-90)),
                                Math.toRadians(-90)
                        )
                        .lineToY(-55 )


                        .setReversed(true)

                        // Return -> Hub: arc UP into hub again
                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(
                                new Pose2d(-24, -24, Math.toRadians(225)),
                                Math.toRadians(135)
                        )

                        //shoot


                        // Hub -> 36,-22
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(
                                new Pose2d(36, -22, Math.toRadians(-90)),
                                Math.toRadians(-90)
                        )

                        .lineToY(-55 )
                        .setReversed(true)

                        // Return -> Hub
                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(
                                new Pose2d(-24, -24, Math.toRadians(225)),
                                Math.toRadians(135)
                        )


                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
