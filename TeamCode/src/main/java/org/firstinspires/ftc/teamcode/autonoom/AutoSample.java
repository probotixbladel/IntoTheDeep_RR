package org.firstinspires.ftc.teamcode.autonoom;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "Auto Sample", group = "Autonomous")
public class AutoSample extends LinearOpMode {
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
        public double liftpick = 550;
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
        public  int finishErection = 125;
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
                    diffl.setPosition(0.49 - 0.09);
                    diffr.setPosition(0.7 - 0.09);
                    liftRight.setTargetPosition((int) liftpick);
                    liftLeft.setTargetPosition((int) liftpick);
                    grabin.setPosition(0.69);
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
                erect.setPower(erectpid.update(-erection, erect.getCurrentPosition())/3);
                espeed = Math.abs(60*(time.seconds()-last_times)*(lasterect - erect.getCurrentPosition()));
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

        public class Grab implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double diflpos = diffleft.getVoltage() / 3.3 * 360;
                double difrpos = diffright.getVoltage() / 3.3 * 360;
                if (grabed) {
                    if (espeed < maxspeed & erection == finishErection & erect.getCurrentPosition() > -finishErection-7 & erect.getCurrentPosition() < -finishErection+7 & 332 < diflpos & diflpos < 342 & 14 < difrpos & difrpos < 24 & liftRight.getCurrentPosition() < liftpick+5 & liftRight.getCurrentPosition() > liftpick-5 & heil.getCurrentPosition() > -8 & heil.getCurrentPosition() < 8) {
                        Actions.runBlocking(new SleepAction(0.4));
                        grab = true;
                        given = true;

                        grabout.setPosition(outClose);
                        grabin.setPosition(0.69);
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
                //double epos = -erect.getCurrentPosition();
                //packet.put("epos", epos);
                //if (epos > 295 & epos < 305 & espeed < maxspeed & !grabed) {
                    //return true;
                //    grabin.setPosition(1);
                //grabed = true;
                //}
                packet.put("uitschuif goto :", erection);
                packet.put("uitschuif pos  :", erect.getCurrentPosition());

                packet.put("heil pos  :", heil.getCurrentPosition());
                packet.put("lift pos  :", liftRight.getCurrentPosition());
                packet.put("difl:", diflpos);
                packet.put("difr:", difrpos );
                //packet.put(espeed < maxspeed & erection == finishErection & erect.getCurrentPosition() > -finishErection-7 & erect.getCurrentPosition() < -finishErection+7 & 332 < diflpos & diflpos < 342 & 14 < difrpos & difrpos < 24 & liftRight.getCurrentPosition(), liftpick+5, liftRight.getCurrentPosition() > liftpick-5, heil.getCurrentPosition() > -8 & heil.getCurrentPosition() < 8);

                return true;
            }
        }
        public Action Grab(){
            return new Grab();
        }

        public class pick implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grabin.setPosition(1);
                return false;
            }
        }
        public Action pick() {
            return new pick();
        }

        public class Upies implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(wristUp);
                grabout.setPosition(outClose);
                liftRight.setTargetPosition((int) toplift);
                liftLeft.setTargetPosition((int) toplift);
                gotoheil = topheil;
                if (liftRight.getCurrentPosition() > toplift - 200 & liftRight.getCurrentPosition() < toplift + 10) {
                    return false;
                }
                return true;
            }
        }
        public Action Upies() {
            return new Upies();
        }
        public class release implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                releaseTime = time.milliseconds();
                erectpid.reset();
                hasReset = false;
                grab = false;
                given = false;
                grabout.setPosition(outOpen);
                Actions.runBlocking(new SleepAction(0.25));
                wrist.setPosition(wristDown);
                liftRight.setTargetPosition((int) liftpick);
                liftLeft.setTargetPosition((int) liftpick);
                gotoheil = 0;
                grabout.setPosition(outOpen);
                onTheway = false;
                diffl.setPosition(0.49 - 0.09);
                diffr.setPosition(0.7 - 0.09);
                return false;
            }
        }
        public Action release() {
            return new release();
        }
        public class release2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                releaseTime = time.milliseconds();
                erectpid.reset();
                hasReset = false;
                grab = false;
                given = false;
                grabout.setPosition(outOpen);
                Actions.runBlocking(new SleepAction(0.25));
                wrist.setPosition(wristDown);
                liftRight.setTargetPosition((int) liftpick);
                liftLeft.setTargetPosition((int) liftpick);
                gotoheil = 0;
                grabout.setPosition(outOpen);
                onTheway = false;
                diffl.setPosition(0.515);
                diffr.setPosition(0.705);
                return false;
            }
        }
        public Action release2() {
            return new release2();
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
                if (erection > -5 & liftRight.getCurrentPosition() < 5 & heil.getCurrentPosition() < 100) {
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
        Pose2d initialPose = new Pose2d(-32.00, -64.00, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        OBJC Objcon = new OBJC(hardwareMap);
        VelConstraint speedv = new VelConstraint() {
            @Override
            public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return 55;
            }
        };
        AccelConstraint speeda = new AccelConstraint() {
            @NonNull
            @Override
            public MinMax minMaxProfileAccel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return new MinMax(-55,55);
            }
        };
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        drive.actionBuilder(initialPose).strafeToLinearHeading(new Vector2d(-50, -50), Math.toRadians(45))
                                                .build(),
                                        Objcon.Upies()
                                ),


                                drive.actionBuilder(new Pose2d(new Vector2d(-50, -50), Math.toRadians(45)))
                                        .strafeToConstantHeading(new Vector2d(-58, -58))
                                        .build(),

                                new  SleepAction(0.25),
                                Objcon.release(),


                                drive.actionBuilder(new Pose2d(new Vector2d(-58, -58), Math.toRadians(45))).strafeToLinearHeading(new Vector2d(-48, -38), Math.toRadians(90))
                                        .build(),
                                drive.actionBuilder(new Pose2d(new Vector2d(-48, -38), Math.toRadians(90)))
                                        .strafeToConstantHeading(new Vector2d(-48, -34))
                                        .build(),
                                Objcon.pick(),
                                new SleepAction(0.25),
                                new ParallelAction(
                                        drive.actionBuilder(new Pose2d(new Vector2d(-48, -34),Math.toRadians(90))).strafeToLinearHeading(new Vector2d(-53, -53), Math.toRadians(45))
                                                .build(),
                                        new SequentialAction(
                                                Objcon.Grab(),
                                                new SleepAction(0.25),
                                                Objcon.Upies()
                                        )
                                ),
                                drive.actionBuilder(new Pose2d(new Vector2d(-53, -53), Math.toRadians(45)))
                                        .strafeToConstantHeading(new Vector2d(-58, -58))
                                        .build(),
                                new  SleepAction(0.25),
                                Objcon.release(),


                                drive.actionBuilder(new Pose2d(new Vector2d(-58, -58), Math.toRadians(45))).strafeToLinearHeading(new Vector2d(-57, -38), Math.toRadians(90))
                                        .build(),
                                drive.actionBuilder(new Pose2d(new Vector2d(-57.25, -38), Math.toRadians(90)))
                                        .strafeToConstantHeading(new Vector2d(-58.75, -36))
                                        .build(),
                                Objcon.pick(),
                                new SleepAction(0.25),
                                new ParallelAction(
                                        drive.actionBuilder(new Pose2d(new Vector2d(-58.75, -36),Math.toRadians(90))).strafeToLinearHeading(new Vector2d(-53, -53), Math.toRadians(45))
                                                .build(),
                                        new SequentialAction(
                                                Objcon.Grab(),
                                                new SleepAction(0.25),
                                                Objcon.Upies()
                                        )
                                ),
                                drive.actionBuilder(new Pose2d(new Vector2d(-53, -53), Math.toRadians(45)))
                                        .strafeToConstantHeading(new Vector2d(-58, -58))
                                        .build(),
                                new  SleepAction(0.25),
                                Objcon.release2(),
                                drive.actionBuilder(new Pose2d(new Vector2d(-58, -58),Math.toRadians(45))).strafeToLinearHeading(new Vector2d(-53, -26), Math.toRadians(180))
                                        .build(),
                                drive.actionBuilder(new Pose2d(new Vector2d(-53, -26), Math.toRadians(180)))
                                        .strafeToConstantHeading(new Vector2d(-58, -26))
                                        .build(),
                                Objcon.pick(),
                                new SleepAction(0.25),
                                drive.actionBuilder(new Pose2d(new Vector2d(-58, -26), Math.toRadians(180)))
                                        .strafeToConstantHeading(new Vector2d(-53, -26))
                                        .build(),
                                new ParallelAction(
                                        drive.actionBuilder(new Pose2d(new Vector2d(-53, -26),Math.toRadians(180))).strafeToLinearHeading(new Vector2d(-53, -53), Math.toRadians(45))
                                                .build(),
                                        new SequentialAction(
                                                Objcon.Grab(),
                                                new SleepAction(0.25),
                                                Objcon.Upies()
                                        )
                                ),
                                drive.actionBuilder(new Pose2d(new Vector2d(-53, -53), Math.toRadians(45)))
                                        .strafeToConstantHeading(new Vector2d(-58, -58))
                                        .build(),
                                //new  SleepAction(0.25),
                                //new ParallelAction(
                                //    drive.actionBuilder(new Pose2d(new Vector2d(-58, -58), Math.toRadians(45)))
                                //        .strafeToLinearHeading(new Vector2d(-24, -48), Math.toRadians(180),speedv,speeda)
                                //        .build(),
                                    Objcon.init()
                                //)
                                //drive.actionBuilder(new Pose2d(new Vector2d(-12, -55), Math.toRadians(180)))
                                //        .splineToConstantHeading(new Vector2d(40, -63), Math.toRadians(180),speedv,speeda)
                                //        .build()

                                ),
                        Objcon.update()
                )
        );
    }
}