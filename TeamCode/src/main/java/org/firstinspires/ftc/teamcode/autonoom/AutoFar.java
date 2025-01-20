package org.firstinspires.ftc.teamcode.autonoom;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous(name = "AutoFar", group = "Autonomous")
public final class AutoFar extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)

                        .splineTo(new Vector2d(-7.88, 34.37), Math.toRadians(-90))
                        .splineTo(new Vector2d(-7.88, 34.58), Math.toRadians(175.29))
                        .splineTo(new Vector2d(-49.09, 40.47), Math.toRadians(-90))
                        .splineTo(new Vector2d(-49.30, 56.23), Math.toRadians(90))
                        .splineTo(new Vector2d(-49.09, 36.05), Math.toRadians(90))
                        .splineTo(new Vector2d(-49.30, 54.97), Math.toRadians(90))
                        .build());


    }
}

