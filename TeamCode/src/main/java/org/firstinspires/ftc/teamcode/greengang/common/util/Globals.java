package org.firstinspires.ftc.teamcode.greengang.common.util;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Globals {
    public static HardwareMap hwMap; // global reference to current hwmap
    public static Telemetry tele; // global reference to current telemetry

    // default alliance is red, but will be changed before auto starts
    public static Alliance alliance = Alliance.RED;
    public static ElapsedTime runtime;

    public static Pose startPose = new Pose(21, 135, Math.toRadians(180));

    public static boolean fieldCentric = true;
    public static boolean autoAimEnabled = false;

    public static double voltage = 13.0;

    public static double ROBOT_CENTER_TO_FRONT = 9;
    public static double GOAL_BACK_TO_FRONT_BLUE = 16.9;
    public static double GOAL_BACK_TO_FRONT_RED = 19.64;

    public static Pose relocalizeBlue = new Pose(18, 120, Math.toRadians(144));
    public static Pose relocalizeRed = new Pose(126, 120, Math.toRadians(36));

    public static String frontLeft = "leftFront";
    public static String frontRight = "rightFront";
    public static String backLeft = "leftBack";
    public static String backRight = "rightBack";

    public static void setAlliance(Alliance alliance) {
        Globals.alliance = alliance;
    }

    public static void flipAlliance() {
        setAlliance(Globals.alliance.flip());
    }

    public static void setVoltage(double voltage) {
        Globals.voltage = voltage;
        Log.i("Globals", "set voltage to " + voltage);
    }

    public static double correctPower(double power) {
        return power * 12.0 / Globals.voltage;
    }

    public static void runtimeTelemetry() {
        tele.addData("Runtime", Globals.runtime.seconds());
    }

    public static double time() {
        return runtime.milliseconds();
    }

    public static double timeSince(double time) {
        return runtime.milliseconds() - time;
    }
}