package org.firstinspires.ftc.teamcode.greengang.common.util;

import org.firstinspires.ftc.teamcode.greengang.common.subsystems.drive.Drivetrain;

public class AprilTagTargeting {

    private Drivetrain drivetrain;

    public AprilTagTargeting(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public double getGoalX() {
        return (Globals.alliance == Alliance.BLUE) ?
                AprilTagMap.TAG20_X :
                AprilTagMap.TAG24_X;
    }

    public double getGoalY() {
        return (Globals.alliance == Alliance.BLUE) ?
                AprilTagMap.TAG20_Y :
                AprilTagMap.TAG24_Y;
    }

    public double getDistanceToGoal() {
        double robotX = drivetrain.pose.position.x;
        double robotY = drivetrain.pose.position.y;
        return Math.hypot(getGoalX() - robotX, getGoalY() - robotY);
    }

    public double getHeadingToGoal() {
        double robotX = drivetrain.pose.position.x;
        double robotY = drivetrain.pose.position.y;
        return Math.atan2(getGoalY() - robotY, getGoalX() - robotX);
    }
}
