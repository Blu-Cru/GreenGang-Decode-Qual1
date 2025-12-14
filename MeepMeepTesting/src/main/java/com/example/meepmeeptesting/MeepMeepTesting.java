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

                        // Start -> Hub: travel direction is basically 45° (straight diagonal), keep it low-curvature
                        .setReversed(false)
                        .setTangent(deg(45))
                        .splineToSplineHeading(
                                new Pose2d(-24, -24, deg(225)),
                                deg(45)
                        )

                        // Hub -> -12,-22: mostly "to the right", finish traveling downward for the lineToY
                        .setReversed(false)
                        .setTangent(deg(0))
                        .splineToSplineHeading(
                                new Pose2d(-12, -22, deg(-90)),
                                deg(-90)
                        )

                        .lineToY(-55)

                        // Return (-12,-55) -> Hub: match the straight-line direction to reduce bending
                        // Vector from (-12,-55) to (-24,-24) is (-12, +31) ~= 112°
                        .setReversed(false)
                        .setTangent(deg(112))
                        .splineToSplineHeading(
                                new Pose2d(-24, -24, deg(225)),
                                deg(112)
                        )

                        // Hub -> +12,-22
                        .setReversed(false)
                        .setTangent(deg(0))
                        .splineToSplineHeading(
                                new Pose2d(12, -22, deg(-90)),
                                deg(-90)
                        )

                        .lineToY(-55)

                        // Return (12,-55) -> Hub: vector (-36, +31) ~= 139°
                        .setReversed(false)
                        .setTangent(deg(139))
                        .splineToSplineHeading(
                                new Pose2d(-24, -24, deg(225)),
                                deg(139)
                        )

                        // Hub -> 36,-22
                        .setReversed(false)
                        .setTangent(deg(0))
                        .splineToSplineHeading(
                                new Pose2d(36, -22, deg(-90)),
                                deg(-90)
                        )

                        .lineToY(-55)

                        // Return (36,-55) -> Hub: vector (-60, +31) ~= 153°
                        .setReversed(false)
                        .setTangent(deg(153))
                        .splineToSplineHeading(
                                new Pose2d(-24, -24, deg(225)),
                                deg(153)
                        )

                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
