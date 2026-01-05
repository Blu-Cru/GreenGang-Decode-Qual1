package org.firstinspires.ftc.teamcode.greengang.opmodes.tele;

import static org.firstinspires.ftc.teamcode.greengang.common.subsystems.shooter.ShooterData.velocityFromDistance;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commonA.intakeA.IntakeA;
import org.firstinspires.ftc.teamcode.commonA.outtakeA.ShooterA;
import org.firstinspires.ftc.teamcode.greengang.opmodes.GreenLinearOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "BlueTele2", group = "TeleOp")
public class Tele extends GreenLinearOpMode {
    Follower follower;
    Limelight3A limelight;
    
    //private VoltageSensor batterySensor;

    public static double TARGET_X = 11;
    public static double TARGET_Y = 127;

    public static double headingP = 2.05;
    public static double headingD = 1.88;
    
//    public static double COEFF_A = 0.045, COEFF_B = 12.5, COEFF_C = 2200.0;
//    public static double HOOD_BASE = 0, HOOD_SLOPE = 0.0001;
//    public static double VELO_GAIN = 1.15, VOLT_NOMINAL = 13.0;
    
    public boolean autoAimToggle = false;

    public enum State {
        START,
        IN, 
        SPIT, 
        OUTTAKE, 
        SHOOT
    }
    StateMachine sm;

    public State state = State.START;
    private double manualRPM = 2100;

    private PDController headingPD;

    private boolean bothTriggers() {
        return gamepad1.left_trigger > 0.1 && gamepad1.right_trigger > 0.1;
    }

    private boolean shootButtons() {
        return gamepad1.right_trigger > 0.1 && gamepad1.right_bumper;
    }

    private boolean rightTrigger() {
        return gamepad1.right_trigger > 0.1;
    }

    private boolean leftTrigger() {
        return gamepad1.left_trigger > 0.1;
    }

    private boolean noInput() {
        return !bothTriggers()
                && !shootButtons()
                && !rightTrigger()
                && !leftTrigger()
                && !gamepad1.x;
    }

    @Override
    public void initialize() {
        addDrivetrain();
        addShooterA();
        addStickyG1();
        addStickyG2();
        addKicker();
        addIntake();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(45, 120, Math.toRadians(180)));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        headingPD = new PDController(headingP, headingD);

        //batterySensor = hardwareMap.voltageSensor.iterator().next();

        sm = new StateMachineBuilder()

                .state(State.START)

                .transition(this::bothTriggers, State.IN)
                .transition(this::shootButtons, State.SHOOT)
                .transition(this::rightTrigger, State.OUTTAKE)
                .transition(this::leftTrigger, State.IN)
                .transition(() -> gamepad1.x, State.SPIT)

                .loop(() -> {

                })

                .state(State.IN)

                .transition(this::bothTriggers, State.IN)
                .transition(this::shootButtons, State.SHOOT)
                .transition(this::rightTrigger, State.OUTTAKE)
                .transition(this::leftTrigger, State.IN)
                .transition(() -> gamepad1.x, State.SPIT)
                .transition(this::noInput, State.START)

                .loop(() -> {

                })

                .state(State.OUTTAKE)

                .transition(this::bothTriggers, State.IN)
                .transition(this::shootButtons, State.SHOOT)
                .transition(this::rightTrigger, State.OUTTAKE)
                .transition(this::leftTrigger, State.IN)
                .transition(() -> gamepad1.x, State.SPIT)
                .transition(this::noInput, State.START)

                .loop(() -> {

                })

                .state(State.SHOOT)

                .transition(this::bothTriggers, State.IN)
                .transition(this::shootButtons, State.SHOOT)
                .transition(this::rightTrigger, State.OUTTAKE)
                .transition(this::leftTrigger, State.IN)
                .transition(() -> gamepad1.x, State.SPIT)
                .transition(this::noInput, State.START)

                .loop(() -> {

                })

                .state(State.SPIT)

                .transition(this::bothTriggers, State.IN)
                .transition(this::shootButtons, State.SHOOT)
                .transition(this::rightTrigger, State.OUTTAKE)
                .transition(this::leftTrigger, State.IN)
                .transition(() -> gamepad1.x, State.SPIT)
                .transition(this::noInput, State.START)

                .loop(() -> {

                })

                .build();

        sm.setState(State.START);
        sm.start();


    }

    @Override
    public void periodic() {

        follower.update();
        Pose currPose = follower.getPose();

        double curX = currPose.getX();
        double curY = currPose.getY();
        double curHeading = currPose.getHeading();

        double dx = TARGET_X - curX;
        double dy = TARGET_Y - curY;

        double vDist = Math.hypot(dx, dy);

        double autoAimTargetVelocity = velocityFromDistance(vDist);

        switch (state) {
            case START:
                intake.setState(IntakeA.State.START);
                outtake.setState(ShooterA.State.START);
                break;
            case IN:
                intake.setState(IntakeA.State.IN);
                outtake.setState(ShooterA.State.START);
                break;
//            case IN:
//                intake.setState(IntakeA.State.IN);
//                outtake.setState(ShooterA.State.SPIN_UP);
//                break;
            case SPIT:
                intake.setState(IntakeA.State.SPIT);
                outtake.setState(ShooterA.State.REVERSE);
                break;
            case OUTTAKE:
                intake.setState(IntakeA.State.START);
                outtake.setState(ShooterA.State.REV);
                break;
            case SHOOT:
                intake.setState(IntakeA.State.START);
                outtake.setState(ShooterA.State.FIRE);
                break;
        }

        //auto aim and drive
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn;

        if (autoAimToggle) {
            double targetHeading = Math.atan2(dy, dx);
            double error = targetHeading - curHeading;

            //clamp between -pi and pi
            error = Math.atan2(Math.sin(error), Math.cos(error));

            turn = headingPD.calculate(error);
            shooterA.setTargetVelocity(autoAimTargetVelocity);
        } else {
            turn = -gamepad1.right_stick_x;

            headingPD.reset();

            if (gamepad1.dpad_left || gamepad2.dpad_left) manualRPM -= 5;
            else if (gamepad1.dpad_right || gamepad2.dpad_right) manualRPM += 5;

            shooterA.setTargetVelocity(manualRPM);
        }

        drivetrain.drive(drive, -strafe, -turn, true);

        telemetry.addData("State", state);
        telemetry.addData("Lock", autoAimToggle ? "ON" : "START");
        telemetry.addData("Dist", "%.2f", vDist);
        telemetry.addData("X/Y", "%.1f, %.1f", curX, curY);
        telemetry.addData("Flywheel Velocity", shooterA.getFlywheelVelocity());
        telemetry.update();
    }

    @Override
    public void telemetry(Telemetry tele) {

    }
}
