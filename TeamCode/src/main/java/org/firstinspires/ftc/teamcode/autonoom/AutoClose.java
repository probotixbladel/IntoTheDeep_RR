package org.firstinspires.ftc.teamcode.autonoom;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

/*
@Autonomous(name = "AutoClose", group = "Autonomous")
public final class AutoClose extends LinearOpMode {
    @Override

    public class Lift {
        private DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "LiftMotor");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        public class LiftUp implements Action {
            private boolean initialized =  false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }
        public class LiftDown implements Action {

        }
    }
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(-12.72, 62.54, Math.toRadians(-90.00));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineToConstantHeading(new Vector2d(0, 36), Math.toRadians(-90.00))
                        .lineToYConstantHeading(38.78)
                        .splineToConstantHeading(new Vector2d(-48.38, 38.78), Math.toRadians(-90))
                        .build());


    }
}
*/
