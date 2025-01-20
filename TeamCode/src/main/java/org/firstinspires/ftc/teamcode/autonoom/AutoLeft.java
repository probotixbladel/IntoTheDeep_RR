package org.firstinspires.ftc.teamcode.autonoom;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Disabled
@Autonomous(name = "AutoLeft", group = "Autonomous")
public final class AutoLeft extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor LiftMotor = null;
        LiftMotor.setPower(1);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setTargetPosition(0);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pose2d beginPose = new Pose2d(0, 0, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        waitForStart();
        /*
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(beginPose)
                        .waitSeconds(5)

                        .strafeTo(new Vector2d(-40, -60))

                        .build()));
        */
    }
}

