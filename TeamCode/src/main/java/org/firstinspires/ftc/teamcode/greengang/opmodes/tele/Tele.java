package org.firstinspires.ftc.teamcode.greengang.opmodes.tele;

import static org.firstinspires.ftc.teamcode.greengang.common.subsystems.shooter.ShooterData.velocityFromDistance;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.ExtendHardstopCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.RetractHardstopCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.StartIntakeCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.StopShooterCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.shoot.KickBallCommand;
import org.firstinspires.ftc.teamcode.greengang.common.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.greengang.common.util.Globals;
import org.firstinspires.ftc.teamcode.greengang.opmodes.GreenLinearOpMode;
import org.firstinspires.ftc.teamcode.greengang.common.util.AprilTagMap;

@Config
@TeleOp(group = "TeleOp")
public class Tele extends GreenLinearOpMode {
    Follower follower;
    Limelight3A limelight;

    //private VoltageSensor batterySensor;


//    public static double COEFF_A = 0.045, COEFF_B = 12.5, COEFF_C = 2200.0;
//    public static double HOOD_BASE = 0, HOOD_SLOPE = 0.0001;
//    public static double VELO_GAIN = 1.15, VOLT_NOMINAL = 13.0;

    public double targetDistance = 0;
    public double autoAimTargetVelocity = 0;


    public double DEADZONE = 0.2;

    public enum State {
        START,
        INTAKE,
        SPIT,
        SPINUP,
        SHOOT
    }
    StateMachine sm;

    public State state = State.START;

    private boolean bothTriggers() {
        return gamepad1.left_trigger > DEADZONE && gamepad1.right_trigger > DEADZONE;
    }

    private boolean rightTrigger() {
        return gamepad1.right_trigger > DEADZONE && gamepad1.left_trigger <= DEADZONE;
    }

    private boolean leftTrigger() {
        return gamepad1.left_trigger > DEADZONE && gamepad1.right_trigger <= DEADZONE;
    }

    private boolean x(){
        return gamepad1.x && gamepad1.left_trigger <= DEADZONE && gamepad1.right_trigger <= DEADZONE;
    }

    private boolean noInput() {
        return !bothTriggers()
                && !rightTrigger()
                && !leftTrigger()
                && !x();
    }

    @Override
    public void initialize() {
        addDrivetrain();
        addShooter();
        addStickyG1();
        addStickyG2();
        addKicker();
        addIntake();

        follower = drivetrain.follower;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        //batterySensor = hardwareMap.voltageSensor.iterator().next();

        sm = new StateMachineBuilder()

                .state(State.START)
                .onEnter(() -> {
                    intake.stop();
                    shooter.stop();
                })
                .transition(this::bothTriggers, State.SHOOT)
                .transition(this::rightTrigger, State.SPINUP)
                .transition(this::leftTrigger, State.INTAKE)
                .transition(this::x, State.SPIT)

                .state(State.INTAKE)
                .onEnter(() -> {
                    new SequentialCommandGroup(
                            new ExtendHardstopCommand(),
                            new StopShooterCommand(),
                            new WaitCommand(100),
                            new StartIntakeCommand()
                    ).schedule();
                })
                .transition(this::bothTriggers, State.SHOOT)
                .transition(this::rightTrigger, State.SPINUP)
                .transition(this::x, State.SPIT)
                .transition(this::noInput, State.START)

                .state(State.SPINUP)
                .onEnter(() -> {
                    intake.stop();
                })
                .loop(() -> {
                    if(Globals.autoAimEnabled){
                        shooter.setTargetVelocity(autoAimTargetVelocity);
                    } else {
                        shooter.setShooterState(Shooter.State.MANUAL);
                    }
                })
                .transition(this::bothTriggers, State.SHOOT)
                .transition(this::leftTrigger, State.INTAKE)
                .transition(this::x, State.SPIT)
                .transition(this::noInput, State.START)

                .state(State.SHOOT)
                .onEnter(() -> {
                    new SequentialCommandGroup(
                            new RetractHardstopCommand(),
                            new WaitCommand(100),
                            new IntakeCommand()
                    ).schedule();
                })

                .loop(() -> {
                    if(Globals.autoAimEnabled){
                        shooter.setTargetVelocity(autoAimTargetVelocity);
                    } else {
                        shooter.setShooterState(Shooter.State.MANUAL);
                    }
                })

                .transition(this::rightTrigger, State.SPINUP)
                .transition(this::leftTrigger, State.INTAKE)
                .transition(this::x, State.SPIT)
                .transition(this::noInput, State.START)

                .state(State.SPIT)
                .onEnter(() -> {
                    shooter.setShooterState(Shooter.State.REVERSE);
                    intake.spit();
                })
                .transition(this::bothTriggers, State.SHOOT)
                .transition(this::rightTrigger, State.SPINUP)
                .transition(this::leftTrigger, State.INTAKE)
                .transition(this::noInput, State.START)
                .build();

        sm.setState(State.START);
        sm.start();
    }

    @Override
    public void periodic() {
        sm.update();
        follower = drivetrain.follower;

        drivetrain.teleOpDrive(gamepad1);

        double x = drivetrain.pose.getX();
        double y = drivetrain.pose.getY();

        double[] distances = AprilTagMap.getDistanceXY(x, y);
        double dx = distances[0];
        double dy = distances[1];

        //measured from front of robot to front of goal for distances
        //15 inches backboard to front + 9 inches center to front of robot

        targetDistance = Math.hypot(dx, dy) - 24;

        autoAimTargetVelocity = velocityFromDistance(targetDistance);

        if(stickyG1.right_bumper){
            new KickBallCommand().schedule();
        }

        if(stickyG1.a) {
            Globals.autoAimEnabled = !Globals.autoAimEnabled;
        }

        if(Globals.autoAimEnabled){
            if (gamepad1.dpad_left || gamepad2.dpad_left) shooter.decreaseVelocity(5);
            else if (gamepad1.dpad_right || gamepad2.dpad_right) shooter.increaseVelocity(5);
        }
    }

    @Override
    public void telemetry(Telemetry tele) {
        tele.addData("State", state);
        tele.addData("Lock", Globals.autoAimEnabled ? "ON" : "START");
        tele.addData("Target Distance", targetDistance);
        tele.addData("X,Y", drivetrain.pose.getX() + "," + drivetrain.pose.getY());

    }
}
