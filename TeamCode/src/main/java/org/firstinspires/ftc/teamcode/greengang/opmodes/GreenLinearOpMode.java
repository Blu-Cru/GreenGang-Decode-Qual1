package org.firstinspires.ftc.teamcode.greengang.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.greengang.common.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.greengang.common.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.greengang.common.subsystems.shooter.Kicker;
import org.firstinspires.ftc.teamcode.greengang.common.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.greengang.common.util.Alliance;
import org.firstinspires.ftc.teamcode.greengang.common.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.greengang.common.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.greengang.common.util.Globals;
import org.firstinspires.ftc.teamcode.greengang.common.util.Robot;

public abstract class GreenLinearOpMode extends LinearOpMode {

    public Robot robot;
    public Drivetrain drivetrain;
    public Shooter shooter;
    public Intake intake;
    public Kicker kicker;
    public StickyGamepad stickyG1;
    public StickyGamepad stickyG2;
    public Alliance alliance;
    public Drive drive;

    double lastTime, loopTimeSum, loopTimeAvg = 0;
    int loopTimeCount;

//    public PinPointLocalizer ppl;

    @Override
    public final void runOpMode() throws InterruptedException {
        Globals.runtime = new ElapsedTime();
        Globals.hwMap = hardwareMap;
        Globals.tele = telemetry;
        CommandScheduler.getInstance().cancelAll();

        stickyG1 = new StickyGamepad(gamepad1);
        stickyG2 = new StickyGamepad(gamepad2);

        robot = Robot.getInstance();
        robot.create(hardwareMap);
        initialize();
        robot.init();

        while(opModeInInit()) {
            if(stickyG1.x) {
                Globals.alliance = Globals.alliance.flip();
            }
            if (stickyG1.left_stick_button){
                Globals.fieldCentric = !Globals.fieldCentric;
            }

            stickyG1.update();
            stickyG2.update();

            // safety for switching controllers
            if(gamepad1.start || gamepad2.start) {
                continue;
            }

            CommandScheduler.getInstance().run();

            // telemetry
            telemetry.addData("ALLIANCE: ", Globals.alliance);
            telemetry.addData("FIELD CENTRIC MODE: ", Globals.fieldCentric);
            telemetry.update();
        }

        waitForStart();
        Globals.runtime = new ElapsedTime();

        onStart();

        while (opModeIsActive()) {
            stickyG1.update();
            stickyG2.update();

            periodic();
            CommandScheduler.getInstance().run();

            robot.update();

            telemetry();
            telemetry(telemetry);
            robot.telemetry(telemetry);
            telemetry.addData("Loop Time", calculateAvgLoopTime());

            telemetry.update();
        }

        end();
        Robot.kill();

    }

    public void enableFTCDashboard() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    // methods to be over written
    public void initialize() {}
    public void initLoop() {}
    public void onStart() {}
    public void periodic() {}
    public void telemetry() {}
    public void end() {}

    // subsystems
    public void addDrivetrain() {drivetrain = robot.addDrivetrain();}
    public void addStickyG1() {stickyG1 = new StickyGamepad(gamepad1);}
    public void addStickyG2() {stickyG2 = new StickyGamepad(gamepad2);}
    public void addShooter() {
        shooter = robot.addShooter();}
    public void addIntake() {intake = robot.addIntake();}
    public void addKicker() {kicker = robot.addKicker();}

    private double calculateAvgLoopTime() {
        loopTimeSum += Globals.runtime.milliseconds() - lastTime;
        lastTime = Globals.runtime.milliseconds();
        loopTimeCount++;

        if(loopTimeCount > 5) {
            loopTimeAvg = loopTimeSum / loopTimeCount;
            loopTimeSum = 0;
            loopTimeCount = 0;
        }

        return loopTimeAvg;
    }

    public abstract void telemetry(Telemetry tele);
}
