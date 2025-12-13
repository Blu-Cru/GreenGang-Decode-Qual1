package org.firstinspires.ftc.teamcode.greengang.opmodes.tele;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.RetractHardstopCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.StopIntakeCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.outtake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.MoveKickerDownCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.StartFlywheelCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.StopLiftingBallCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.StopShooterCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.shoot.ShootCommand;
import org.firstinspires.ftc.teamcode.greengang.common.util.AprilTagTargeting;
import org.firstinspires.ftc.teamcode.greengang.common.util.AutoLock;
import org.firstinspires.ftc.teamcode.greengang.common.util.Globals;
import org.firstinspires.ftc.teamcode.greengang.opmodes.GreenLinearOpMode;

@TeleOp(name="Main FSM", group ="TeleOp")

public class MainFSM extends GreenLinearOpMode {
    enum State{
        DEFAULT,
        INTAKE,
        OUTTAKE,
        SHOOT,
    }

    StateMachine sm;
    AutoLock autoLock;
    AprilTagTargeting targeting;
    Limelight3A limelight;

    public void initialize(){
        addDrivetrain();
        addShooter();
        addStickyG1();
        addStickyG2();
        addKicker();
        addIntake();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        autoLock = new AutoLock(limelight, drivetrain);
        targeting = new AprilTagTargeting(drivetrain);

        sh.setTargeting(targeting);
        autoLock.setTargeting(targeting);

        sm = new StateMachineBuilder()

                //default
                .state(State.DEFAULT)
                .onEnter(() -> {})
                .transition(() -> stickyG2.a, State.INTAKE, () -> {
                    new IntakeCommand().schedule();
                })
                .transition(() -> stickyG2.b, State.OUTTAKE, () -> {
                    new OuttakeCommand().schedule();
                })
                .transition(() -> stickyG2.dpad_down, State.SHOOT, () -> {
                    new SequentialCommandGroup(
                            new StartFlywheelCommand(),
                            new ShootCommand()
                    ).schedule();
                })

                //intake
                .state(State.INTAKE)
                .transition(() -> stickyG2.right_bumper, State.DEFAULT, () -> {
                    new SequentialCommandGroup(
                            new StopIntakeCommand()
                    ).schedule();
                })
                .transition(() -> stickyG2.dpad_down, State.SHOOT, () -> {
                    new SequentialCommandGroup(
                            new StopIntakeCommand(),
                            new StartFlywheelCommand(),
                            new ShootCommand()
                    ).schedule();
                })
                .transition(() -> stickyG2.b, State.OUTTAKE, () -> {
                    new SequentialCommandGroup(
                            new OuttakeCommand()
                    ).schedule();
                })

                //outtake
                .state(State.OUTTAKE)
                .transition(() -> stickyG2.right_bumper, State.DEFAULT, () -> {
                    new SequentialCommandGroup(
                            new StopIntakeCommand()
                    ).schedule();
                })
                .transition(() -> stickyG2.a, State.INTAKE, () -> {
                    new SequentialCommandGroup(
                            new IntakeCommand()
                    ).schedule();
                })

                //shoot command
                .state(State.SHOOT)
                .transition(() -> stickyG2.right_bumper, State.DEFAULT, () -> {
                    new SequentialCommandGroup(
                            new StopIntakeCommand(),
                            new StopLiftingBallCommand(),
                            new MoveKickerDownCommand()
                    ).schedule();
                })
                .transition(() -> stickyG2.a, State.INTAKE, () -> {
                    new SequentialCommandGroup(
                            new IntakeCommand()
                    ).schedule();
                })

                .onExit(() -> {
                    new SequentialCommandGroup(
                            new StopShooterCommand()
                    ).schedule();
                })

                .build();


        sm.setState(State.DEFAULT);
        sm.start();
    }

    @Override
    public void telemetry(Telemetry tele){
        tele.addData("Robot State: ", sm.getState());
        tele.addData("Auto Aim", Globals.autoAimEnabled ? "AUTO" : "MANUAL");
    }

    @Override
    public void periodic(){
        drivetrain.teleOpDrive(gamepad1);

        if (stickyG1.right_stick_button){
            Globals.autoAimEnabled = !Globals.autoAimEnabled;

            if(Globals.autoAimEnabled){
                gamepad1.setLedColor(0, 255, 0, 500);
                gamepad1.rumbleBlips(2);
            } else{
                gamepad1.setLedColor(255, 255, 0, 500);
                gamepad1.rumbleBlips(1);
            }
        }

        if(gamepad1.a && Globals.autoAimEnabled) {
            autoLock.move();
            gamepad1.rumble(150);
        }

        sm.update();
        telemetry.update();
    }

}