package org.firstinspires.ftc.teamcode.autonoom;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;

import com.qualcomm.robotcore.hardware.Servo;

import java.lang.Math;
import java.util.Arrays;

@Config
@Autonomous(name = "Auto Specimen", group = "Autonomous")
public class AutoSpecimen extends LinearOpMode {
    public class OBJC {
        private ElapsedTime time = new ElapsedTime();
        private double last_time = 0;
        private double last_times = 0;
        private DcMotor erect = null;
        private DcMotor liftLeft = null;
        private DcMotor liftRight = null;
        private DcMotorEx heil = null;
        private Servo diffl = null;
        private Servo diffr = null;
        private Servo grabin = null;
        private Servo wrist = null;
        private Servo grabout = null;
        private boolean lasta = false;
        private double lasterect = 0;
        private double espeed = 0;
        private boolean onTheway = false;
        public double maxspeed = 3;
        public double wristDown = 0.32;
        public double wristUp = 0.025;
        public double wristFin =  0.45;
        public double wait = 250;
        public double liftpick = 530;
        public  double kp = 0.045;
        public  double ki = 0.01;
        public  double kd = 0.0002;
        public double kf = 0.1;
        public double erectspeed = 25.0;
        public double outClose = 0.67;
        public double outOpen = 0.45;
        public double gotoheil = 0;
        public double toplift = 2900;
        public double topheil = 500;
        private final double ticksInDegree = 450 / 180.0;
        private double releaseTime = 0;
        public  boolean grab = false;
        private int erection = 55;
        public  int finishErection = 130;
        public  int max_fps = 30;
        private double pos = 0;
        private boolean given = false;
        private boolean hasReset = false;
        private boolean grabed = false;
        //public static double maxspeed = 20;
        public   double max_heil = 0.5;
        AnalogInput diffleft = null;
        AnalogInput diffright = null;
        AnalogInput wristAngle = null;
        public  int liftw = 30;
        public  int liftwait = 5;
        public  int heilfin = 125;
        public  int liftfin = 1500;
        public double i_tollerance = 10;
        private boolean in_range = false;
        private boolean finished = false;
        private boolean done = false;
        private PIDController erectpid = new PIDController(0.01,0.0,0.0005);
        private PIDController heilpid = new PIDController(0.008, 0.0, 0.0005);

        public OBJC(HardwareMap hardwareMap) {
            diffleft = hardwareMap.get(AnalogInput.class, "diffleft");
            diffright = hardwareMap.get(AnalogInput.class, "diffright");
            wristAngle = hardwareMap.get(AnalogInput.class, "wristAngle");
            liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
            liftRight = hardwareMap.get(DcMotor.class, "liftRight");
            erect = hardwareMap.get(DcMotor.class, "erect");
            heil = hardwareMap.get(DcMotorEx.class, "heil");
            diffl = hardwareMap.get(Servo.class, "diffLeft");
            diffr = hardwareMap.get(Servo.class, "diffRight");
            grabin = hardwareMap.get(Servo.class, "grab'in");
            wrist = hardwareMap.get(Servo.class, "wrist");
            grabout = hardwareMap.get(Servo.class, "grab'out");
            erect.setPower(0);
            erect.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            erect.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            erect.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            heil.setPower(0);
            heil.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            heil.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            heil.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftRight.setTargetPosition(0);
            liftRight.setPower(1);
            liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftLeft.setPower(1);
            liftLeft.setTargetPosition(0);
            liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class update implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    erection = 55;
                    grabin.setPosition(1);
                    diffl.setPosition(0.49);
                    diffr.setPosition(0.7);
                    erect.setPower(0.5);
                    liftRight.setTargetPosition((int) liftpick);
                    liftLeft.setTargetPosition((int) liftpick);
                    initialized = true;
                    grabed = true;
                }

                if (Math.abs(gotoheil - heil.getCurrentPosition()) < i_tollerance) {
                    if (!in_range) {
                        in_range = true;
                        heilpid.Ri(0.01);
                    }
                    heilpid.setParams(0.015,0.01,0.00055);
                } else {
                    if (in_range) {
                        in_range = false;
                        heilpid.Ri(0);
                    }
                    heilpid.setParams(0.015,0,0.00055);
                }
                heil.setPower(Math.max(-max_heil, Math.min(max_heil,(Math.sin(Math.toRadians(gotoheil / ticksInDegree + 40)) * 0.1) + heilpid.update(gotoheil, heil.getCurrentPosition()))));
                if (done){
                    heil.setPower(-0.4);
                    erect.setPower(0.4);
                }
                //erect.setPower(erectpid.update(-erection, erect.getCurrentPosition())/3);
                //espeed = Math.abs(60*(time.seconds()-last_times)*(lasterect - erect.getCurrentPosition()));
                last_times = time.seconds();
                lasterect = erect.getCurrentPosition();
                packet.put("espeed", espeed);
                if (finished) {
                    return false;
                }
                return true;
            }
        }
        public Action update() {
            return new update();
        }
        /*
        public class Give implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double diflpos = diffleft.getVoltage() / 3.3 * 360;
                double difrpos = diffright.getVoltage() / 3.3 * 360;
                if (grabed) {
                    if (espeed < maxspeed & erection == finishErection & erect.getCurrentPosition() > -finishErection-7 & erect.getCurrentPosition() < -finishErection+7 & 332 < diflpos & diflpos < 342 & 14 < difrpos & difrpos < 24 & liftRight.getCurrentPosition() < liftpick+5 & liftRight.getCurrentPosition() > liftpick-5 & heil.getCurrentPosition() > -8 & heil.getCurrentPosition() < 8) {
                        Actions.runBlocking(new SleepAction(0.15));
                        grab = true;
                        given = true;

                        grabout.setPosition(outClose);
                        grabin.setPosition(0.69);
                        Actions.runBlocking(new SleepAction(0.22));
                        erection = 0;
                        diffl.setPosition(0.515);
                        diffr.setPosition(0.705);
                        return false;
                    }
                    if (!given) {
                        if (grab) {
                            grabout.setPosition(outClose);
                            grabin.setPosition(0.69);
                        }
                        if (!grab) {
                            grabout.setPosition(outOpen);
                            grabin.setPosition(1);
                        }
                        wrist.setPosition(wristDown);
                        liftRight.setTargetPosition((int) liftpick);
                        liftLeft.setTargetPosition((int) liftpick);
                        diffl.setPosition(0.02);
                        diffr.setPosition(1.0);

                        if (!hasReset) {
                            erectpid.setParams(0.02, 0.008, 0.001);
                            erectpid.reset();
                            hasReset = true;
                        }

                        if (332 < diflpos & diflpos < 342 & 14 < difrpos & difrpos < 24 & espeed < maxspeed & erection != finishErection) {
                            erection = finishErection;
                            erectpid.reset();
                            //erectpid.setParams(0.01,0.00025,0.00008);
                            erectpid.setParams(kp, ki, kd);
                        } else if (erection != finishErection) {
                            erection = 215;
                        }
                    }
                }
                return true;
            }
        }
        public Action Give(){
            return new Give();
        }
        */
        public class redef implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                erect.setPower(1);
                erect.setTargetPosition(0);
                erect.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                erect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                erect.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                return false;
            }
        }
        public Action redef() {
            return new redef();
        }
        public class pick implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grabout.setPosition(outClose);
                return false;
            }
        }
        public Action pick() {
            return new pick();
        }

        public class release implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grabout.setPosition(outOpen);
                return false;
            }
        }
        public Action release() {
            return new release();
        }

        public class pickup implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                liftLeft.setTargetPosition(liftw);
                liftRight.setTargetPosition(liftw);
                grabout.setPosition(outOpen);
                gotoheil = 665;
                wrist.setPosition(wristDown);
                return false;
            }
        }
        public Action pickup() {
            return new pickup();
        }

        public class deliver implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grabout.setPosition(outClose);
                wrist.setPosition(wristFin);

                liftLeft.setTargetPosition(liftfin);
                liftRight.setTargetPosition(liftfin);
                gotoheil = heilfin;
                if (liftRight.getCurrentPosition() > liftfin -1000 & gotoheil-50<heil.getCurrentPosition() & gotoheil+50 > heil.getCurrentPosition()) {
                    return false;
                }
                return true;
            }
        }
        public Action deliver() {
            return new deliver();
        }
        public class init implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                releaseTime = time.milliseconds();
                erectpid.reset();
                done = true;
                hasReset = false;
                grab = false;
                given = false;
                erection = 0;
                grabout.setPosition(outOpen);
                Actions.runBlocking(new SleepAction(0.25));
                wrist.setPosition(wristUp);
                liftRight.setTargetPosition(0);
                liftLeft.setTargetPosition(0);
                gotoheil = 100;
                grabout.setPosition(outOpen);
                onTheway = false;
                diffl.setPosition(0.515);
                diffr.setPosition(0.705);
                if (liftRight.getCurrentPosition() < 5 & heil.getCurrentPosition() < 100) {
                    finished = true;
                    return false;
                }
                return true;
            }
        }
        public Action init() {
            return new init();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(8, -64.00, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        OBJC Objcon = new OBJC(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        VelConstraint speedv = new VelConstraint() {
            @Override
            public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return 60;
            }
        };
        AccelConstraint speeda = new AccelConstraint() {
            @NonNull
            @Override
            public MinMax minMaxProfileAccel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return new MinMax(-50,50);
            }
        };



        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        new SequentialAction(
                                                new SleepAction(0.1),
                                                drive.actionBuilder(initialPose)
                                                         .strafeToConstantHeading(new Vector2d(-6, -31))
                                                         .build()
                                       ),
                                       Objcon.deliver()

                                ),
                                Objcon.release(),
                                Objcon.pickup(),
                                drive.actionBuilder(new Pose2d(new Vector2d(-6, -31.5), Math.toRadians(90)))
                                        .splineToConstantHeading(new Vector2d(0, -44),Math.toRadians(90),speedv)
                                        //.splineToConstantHeading(new Vector2d(33, -40), Math.toRadians(0),speedv,speeda)
                                        //.splineToConstantHeading(new Vector2d(45, -25), Math.toRadians(90))//,speedv,speeda)
                                        .splineToConstantHeading(new Vector2d(28., -46.98), Math.toRadians(0))
                                        .splineToConstantHeading(new Vector2d(42, -20), Math.toRadians(0))
                                        .splineToConstantHeading(new Vector2d(48, -18), Math.toRadians(0))
                                        .splineToConstantHeading(new Vector2d(48, -18), Math.toRadians(0))
                                        .strafeToConstantHeading(new Vector2d(48, -47),speedv,speeda)
                                        .splineToConstantHeading(new Vector2d(53, -20), Math.toRadians(90))
                                        .splineToConstantHeading(new Vector2d(58, -18), Math.toRadians(180))
                                        .strafeToConstantHeading(new Vector2d(58, -47),speedv,speeda)
                                        //.splineToConstantHeading(new Vector2d(58, -22), Math.toRadians(90))
                                        //.splineToConstantHeading(new Vector2d(66, -19), Math.toRadians(180))
                                        //.strafeToConstantHeading(new Vector2d(66, -50),speedv,speeda)
                                        //.strafeToConstantHeading(new Vector2d(66, -50),speedv,speeda)
                                        .strafeToConstantHeading(new Vector2d(38, -61 ))
                                        //.splineToConstantHeading(new Vector2d(50, -15), Math.toRadians(0),speedv,speeda)
                                        .build(),
                                new SleepAction(0.1),
                                Objcon.pick(),
                                new SleepAction(0.25),
                                new ParallelAction(
                                    Objcon.deliver(),
                                    drive.actionBuilder(new Pose2d(new Vector2d(38, -61),Math.toRadians(90)))
                                            .strafeToConstantHeading(new Vector2d(-2, -38),speedv,speeda)
                                            .strafeToConstantHeading(new Vector2d(-2, -31.5),speedv,speeda)
                                            .build()
                                ),
                                new SleepAction(0.1),
                                Objcon.release(),
                                Objcon.pickup(),
                                drive.actionBuilder(new Pose2d(new Vector2d(2, -34),Math.toRadians(90)))
                                        .strafeToConstantHeading(new Vector2d(38, -60.5),speedv,speeda)
                                        .build(),
                                new SleepAction(0.2),
                                Objcon.pick(),
                                new SleepAction(0.25),
                                new ParallelAction(
                                        Objcon.deliver(),
                                        drive.actionBuilder(new Pose2d(new Vector2d(38, -61),Math.toRadians(90)))
                                                .strafeToConstantHeading(new Vector2d(1, -38),speedv,speeda)
                                                .strafeToConstantHeading(new Vector2d(1, -31.5),speedv,speeda)
                                            .build()
                                ),
                                new SleepAction(0.1),
                                Objcon.release(),

                                Objcon.pickup(),
                                drive.actionBuilder(new Pose2d(new Vector2d(2, -34),Math.toRadians(90)))
                                        .strafeToConstantHeading(new Vector2d(38, -60),speedv,speeda)
                                        .build(),
                                new SleepAction(0.2),
                                Objcon.pick(),
                                new SleepAction(0.25),
                                new ParallelAction(
                                        Objcon.deliver(),
                                        drive.actionBuilder(new Pose2d(new Vector2d(38, -61),Math.toRadians(90)))
                                                .strafeToConstantHeading(new Vector2d(3, -38),speedv,speeda)
                                                .strafeToConstantHeading(new Vector2d(3, -31.5),speedv,speeda)
                                                .build()
                                ),
                                new SleepAction(0.1),
                                Objcon.release(),
                                new ParallelAction(
                                        drive.actionBuilder(new Pose2d(new Vector2d(3, -31.5),Math.toRadians(90)))
                                                .strafeToConstantHeading(new Vector2d(62, -62),speedv,speeda)
                                                .build(),
                                        new SequentialAction(
                                            new SleepAction(0.5),
                                            Objcon.init()
                                        )
                                )

                        ),
                        Objcon.update()
                )
        );
    }
}