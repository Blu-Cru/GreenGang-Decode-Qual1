package org.firstinspires.ftc.teamcode.greengang.opmodes.tele;

import static org.firstinspires.ftc.teamcode.greengang.common.subsystems.shooter.ShooterData.velocityFromDistance;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.controller.PDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.ExtendHardstopCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.StartIntakeCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.StopIntakeCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.MoveKickerUpCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.SetFlywheelVelocityCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.spit.SpitCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.StopShooterCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.SwitchFlywheelStateCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.shoot.KickBallCommand;
import org.firstinspires.ftc.teamcode.greengang.common.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.greengang.common.util.Alliance;
import org.firstinspires.ftc.teamcode.greengang.common.util.Globals;
import org.firstinspires.ftc.teamcode.greengang.common.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.greengang.opmodes.GreenLinearOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "BlueTele2", group = "TeleOp")
public class Tele extends GreenLinearOpMode {
    Follower follower;
    Limelight3A limelight;
    
    //private VoltageSensor batterySensor;

    public static double TARGET_BLUE_X = 11;
    public static double TARGET_BLUE_Y = 127;

    public static double TARGET_RED_X = 11;
    public static double TARGET_RED_Y = -127;

    public static double headingP = 2.05;
    public static double headingD = 1.88;
    
//    public static double COEFF_A = 0.045, COEFF_B = 12.5, COEFF_C = 2200.0;
//    public static double HOOD_BASE = 0, HOOD_SLOPE = 0.0001;
//    public static double VELO_GAIN = 1.15, VOLT_NOMINAL = 13.0;
    
    public boolean autoAimToggle = false;
    public double targetDistance = 0;
    public double autoAimTargetVelocity = 0;

    public enum State {
        START,
        INTAKE,
        SPIT, 
        SPINUP,
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
        addShooter();
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
                .onEnter(() -> {
                    intake.stop();
                    sh.stop();
                })
                .transition(this::shootButtons, State.SHOOT)
                .transition(this::bothTriggers, State.INTAKE)
                .transition(this::rightTrigger, State.SPINUP)
                .transition(this::leftTrigger, State.INTAKE)
                .transition(() -> gamepad1.x, State.SPIT)

                .state(State.INTAKE)
                .onEnter(() -> {
                    intake.in();
                    sh.stop();
                })
                .transition(this::shootButtons, State.SHOOT)
                .transition(this::rightTrigger, State.SPINUP)
                .transition(() -> gamepad1.x, State.SPIT)
                .transition(this::noInput, State.START)

                .state(State.SPINUP)
                .onEnter(() -> {
                    intake.stop();
                })
                .loop(() -> {
                    if(autoAimToggle){
                        sh.setTargetVelocity(autoAimTargetVelocity);
                    } else {
                        sh.setTargetVelocity(manualRPM);
                    }
                })
                .transition(this::shootButtons, State.SHOOT)
                .transition(this::bothTriggers, State.INTAKE)
                .transition(this::leftTrigger, State.INTAKE)
                .transition(() -> gamepad1.x, State.SPIT)
                .transition(this::noInput, State.START)

                .state(State.SHOOT)
                .onEnter(() -> {
                    intake.in();
                })
                .loop(() -> {
                    if(autoAimToggle){
                        sh.setTargetVelocity(autoAimTargetVelocity);
                    } else {
                        sh.setTargetVelocity(manualRPM);
                    }
                })

                .transition(this::bothTriggers, State.INTAKE)
                .transition(this::rightTrigger, State.SPINUP)
                .transition(this::leftTrigger, State.INTAKE)
                .transition(() -> gamepad1.x, State.SPIT)
                .transition(this::noInput, State.START)

                .state(State.SPIT)
                .onEnter(() -> {
                    sh.setShooterState(Shooter.State.REVERSE);
                    intake.spit();
                })
                .transition(this::shootButtons, State.SHOOT)
                .transition(this::bothTriggers, State.INTAKE)
                .transition(this::rightTrigger, State.SPINUP)
                .transition(this::leftTrigger, State.INTAKE)
                .transition(this::noInput, State.START)
                .build();

        sm.setState(State.START);
        sm.start();
    }

    @Override
    public void periodic() {
        follower.update();

        double dx, dy;

        if(Globals.alliance == Alliance.BLUE) {
            dx = TARGET_BLUE_X - drivetrain.pose.position.x;
            dy = TARGET_BLUE_Y - drivetrain.pose.position.y;
        } else {
            dx = TARGET_RED_X - drivetrain.pose.position.x;
            dy = TARGET_RED_Y - drivetrain.pose.position.y;
        }

        targetDistance = Math.hypot(dx, dy);

        autoAimTargetVelocity = velocityFromDistance(targetDistance);

        //auto aim and drive
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn;

        if (autoAimToggle) {
            double targetHeading = Math.atan2(dy, dx);
            double error = targetHeading - drivetrain.heading;

            //clamp between -pi and pi
            error = Math.atan2(Math.sin(error), Math.cos(error));

            turn = headingPD.calculate(error);
        } else {
            turn = -gamepad1.right_stick_x;

            headingPD.reset();

            if (gamepad1.dpad_left || gamepad2.dpad_left) manualRPM -= 5;
            else if (gamepad1.dpad_right || gamepad2.dpad_right) manualRPM += 5;
        }

        drivetrain.drive(drive, -strafe, -turn, true);


        if(stickyG1.a){
            new KickBallCommand().schedule();
        }
    }

    @Override
    public void telemetry(Telemetry tele) {
        tele.addData("State", state);
        tele.addData("Lock", autoAimToggle ? "ON" : "START");
        tele.addData("Target Distance", targetDistance);
        tele.addData("X,Y", drivetrain.pose.position.x + "," + drivetrain.pose.position.y);
        tele.addData("Flywheel Velocity", shooterA.getFlywheelVelocity());
    }
}
