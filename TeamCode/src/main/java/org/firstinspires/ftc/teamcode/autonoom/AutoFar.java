package org.firstinspires.ftc.teamcode.autonoom;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.VelConstraint;
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
    public class Lift {
        private DcMotorEx lift;
        private DcMotorEx ArmMotor;
        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "LiftMotor");
            ArmMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.REVERSE);
            ArmMotor.setTargetPosition(0);
            ArmMotor.setPower(1);
            ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(1.0);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2150.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 0.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }






    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        waitForStart();
        Lift lift = new Lift(hardwareMap);
        /*
        Actions.runBlocking(lift.liftUp());
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineToConstantHeading(new Vector2d(30.00, 8.48), Math.toRadians(0.00))
                        .build());
        */
        VelConstraint vel = new VelConstraint() {
            @Override
            public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return 50;
            }
        };
        AccelConstraint acc = new AccelConstraint() {
            @NonNull
            @Override
            public MinMax minMaxProfileAccel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return new MinMax(-50,50);
            }
        };

        Actions.runBlocking(new ParallelAction(
                lift.liftUp(),
                drive.actionBuilder(beginPose)
                        .splineToConstantHeading(new Vector2d(30.00, 8.48), Math.toRadians(0.00))
                        .build()
                )
        );
        Actions.runBlocking(lift.liftDown());
        Actions.runBlocking(drive.actionBuilder(new Pose2d(30.00, 8.48, 0.00) )
                        //.lineToXConstantHeading(23)
                        .strafeToConstantHeading(new Vector2d(23.00, 8.48))
                        .splineToConstantHeading(new Vector2d(23.86, -8.93), Math.toRadians(0.00))//-88.39))

                        .splineToConstantHeading(new Vector2d(45.00, -25.00), Math.toRadians(0.00))

                        //.lineToYConstantHeading( -35.00)
                        .strafeToConstantHeading(new Vector2d(45.00, -34.16))
                        //.lineToXConstantHeading( 8.50)
                        .strafeToConstantHeading(new Vector2d(8.50, -35.00), vel, acc)

                        //.strafeToConstantHeading( 43.50)
                        .strafeToConstantHeading(new Vector2d(45.00, -34.16))
                        //.lineToYConstantHeading( -45.00)
                        .strafeToConstantHeading(new Vector2d(45.00, -45.00))
                        //.lineToXConstantHeading( 8.50)
                        .strafeToConstantHeading(new Vector2d(8.50, -45.00))

                        //.lineToXConstantHeading( 43.50)
                        .strafeToConstantHeading(new Vector2d(45.00, -45.00))
                        //.lineToYConstantHeading( -53.00)
                        .strafeToConstantHeading(new Vector2d(43.50, -53.00))
                        //.lineToXConstantHeading( 8.50)
                        .strafeToConstantHeading(new Vector2d(5.50, -51.00))


                        .build()
                );




    }
}

