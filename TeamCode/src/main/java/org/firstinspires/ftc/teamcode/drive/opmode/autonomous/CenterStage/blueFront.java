package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;





@Autonomous(name = "blueFront auto")
@Config
public class blueFront extends LinearOpMode {

    // beginning placement of robot
    private final Pose2d blueFrontStart = new Pose2d(12,60, Math.toRadians(-90));
    private final Pose2d blueFrontEnd = new Pose2d(48, 60, Math.toRadians(0));

    // drive forward 2 feet
    private final Pose2d centerOfLines = new Pose2d(12, 36, Math.toRadians(-90));

    // the poses for the lines on the ground

    private final Pose2d leftLine = new Pose2d(0, 36, Math.toRadians(-180));
    private final Pose2d rightLine = new Pose2d(24, 36, Math.toRadians(0));
    private final Pose2d centerLine = new Pose2d(12, 24, Math.toRadians(-90));


    private Pose2d _pose;

    SampleMecanumDrive drive;


    @Override
    public void runOpMode() throws InterruptedException {
        drive.initArm();

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(blueFrontStart);
        TrajectorySequence placePurpleFront = drive.trajectorySequenceBuilder(_pose)
                .lineToSplineHeading(blueFrontEnd)
                .build();
        drive.followTrajectorySequence(placePurpleFront);

    }
}