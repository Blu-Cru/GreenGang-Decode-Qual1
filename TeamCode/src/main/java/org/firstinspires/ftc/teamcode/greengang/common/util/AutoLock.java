package org.firstinspires.ftc.teamcode.greengang.common.util;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.greengang.common.subsystems.drive.DrivetrainOLD;

import java.util.List;

public class AutoLock implements GreenSubsystem, Subsystem {
    public DrivetrainOLD drivetrainOLD;
    public Limelight3A limelight;
    AprilTagTargeting targeting;
    LLResult result;

    public AutoLock(Limelight3A limelight, DrivetrainOLD drivetrainOLD){
        this.drivetrainOLD = drivetrainOLD;
        this.limelight = limelight;
    }

    public void setTargeting(AprilTagTargeting targeting) {
        this.targeting = targeting;
    }

    @Override
    public void init() {

    }

    public void move(){
        result = limelight.getLatestResult();

        if(result != null && result.isValid()){
            List<LLResultTypes.FiducialResult> results = result.getFiducialResults();
            LLResultTypes.FiducialResult targetTag = null;

            for(LLResultTypes.FiducialResult res : results){
                int id = res.getFiducialId();

                boolean blue = id == 20 && Globals.alliance == Alliance.BLUE;
                boolean red = id == 24 && Globals.alliance == Alliance.RED;

                if(blue || red){
                    targetTag = res;
                    break;
                }
            }

            if(targetTag != null){
                double xError = targetTag.getTargetXDegrees();

                drivetrainOLD.pid.setTargetHeading(drivetrainOLD.heading - Math.toRadians(xError));
                return;
            }
        }

        double heading = targeting.getHeadingToGoal();

        drivetrainOLD.pid.setTargetHeading(heading);
    }

    public void relocalize(){
        result = limelight.getLatestResult();

        if(result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> results = result.getFiducialResults();

            for (LLResultTypes.FiducialResult res : results) {
                int id = res.getFiducialId();

                if (id == 20 || id == 24) {
                    drivetrainOLD.relocalizeFromLimelight(result.getBotpose());
                    break;
                }
            }
        }
    }

    @Override
    public void update() {}

    @Override
    public void telemetry(Telemetry telemetry) {

    }
}
