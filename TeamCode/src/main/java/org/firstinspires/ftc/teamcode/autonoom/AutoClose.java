package org.firstinspires.ftc.teamcode.autonoom;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous(name = "AutoClose", group = "Autonomous")
public final class AutoClose extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(-12.72, 62.54, Math.toRadians(-90.00));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineToConstantHeading(new Vector2d(0, 36), Math.toRadians(-90.00))
                        .splineToConstantHeading(new Vector2d(-48.38, 38.78), Math.toRadians(-90))
                        .build());




    }
    }

