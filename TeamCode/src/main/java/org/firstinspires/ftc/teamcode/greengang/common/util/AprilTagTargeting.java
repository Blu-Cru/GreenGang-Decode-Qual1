package org.firstinspires.ftc.teamcode.greengang.common.util;

import org.firstinspires.ftc.teamcode.greengang.common.subsystems.drive.DrivetrainOLD;

public class AprilTagTargeting {

    private DrivetrainOLD drivetrainOLD;

    public AprilTagTargeting(DrivetrainOLD drivetrainOLD) {
        this.drivetrainOLD = drivetrainOLD;
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
        double robotX = drivetrainOLD.pose.position.x;
        double robotY = drivetrainOLD.pose.position.y;
        return Math.hypot(getGoalX() - robotX, getGoalY() - robotY);
    }

    public double getHeadingToGoal() {
        double robotX = drivetrainOLD.pose.position.x;
        double robotY = drivetrainOLD.pose.position.y;
        return Math.atan2(getGoalY() - robotY, getGoalX() - robotX);
    }
}
