/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;


import org.firstinspires.ftc.teamcode.MecanumDrive;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="TeleopDrive", group="Linear OpMode")
//@Disabled
public class TeleopDrive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor LiftMotor = null;
    private DcMotor ArmMotor = null;
    private Servo GrabServo = null;
    private double gearshift = 0.25;
    private double turnspeed = 0.4;
    //private double LiftPos = 0;

    private double controller(double x) {
        return (Math.pow(Math.abs(x) * 0.75 + 0.25, 2.3) * Math.signum(x));
    }

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "wheelLeftFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "wheelLeftRear");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "wheelRightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "wheelRightRear");

        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");

        GrabServo = hardwareMap.get(Servo.class, "GrabServo");

        LiftMotor.setPower(0.4);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setTargetPosition(0);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ArmMotor.setPower(0.4);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setTargetPosition(0);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        GrabServo.setPosition(0.0);

        
        //LiftEncoder new OverflowEncoder(new RawEncoder(LiftMotor));

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        ArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        boolean LiftUp = false;
        boolean ArmOut = false;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            /*
            double axial   = -gamepad1.left_stick_y * gearshift; ; // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x * gearshift;  
            double yaw     =  gamepad1.right_stick_x * turnspeed; 
            */

            double axial   = -controller(gamepad1.left_stick_y) * gearshift; // Note: pushing stick forward gives negative value
            double lateral =  controller(gamepad1.left_stick_x) * gearshift;             
            double yaw     =  controller(gamepad1.right_stick_x) * turnspeed;


            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = (axial + lateral + yaw);// * gearshift;
            double rightFrontPower = (axial - lateral - yaw);// * gearshift;
            double leftBackPower   = (axial - lateral + yaw);// * gearshift;
            double rightBackPower  = (axial + lateral - yaw);// * gearshift;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            //LIFT
            if (gamepad2.dpad_up) {
                LiftMotor.setTargetPosition(2150);   //2600
                LiftMotor.setPower(0.8);
                LiftUp = true;
            } else if (gamepad2.dpad_right) {
                LiftMotor.setTargetPosition(1300);
                LiftMotor.setPower(0.6);
                LiftUp = true;
            } else if (gamepad2.dpad_down && !ArmOut) {
                LiftMotor.setTargetPosition(300);
                LiftMotor.setPower(0.4);
                LiftUp = false;
            }

            //ARMMOTOR
            if(LiftUp) {
                if (gamepad2.left_trigger == 1) {
                    ArmMotor.setTargetPosition(0);
                    ArmMotor.setPower(0.12);
                    ArmOut = false;
                } else if (gamepad2.right_trigger == 1) {
                    ArmMotor.setTargetPosition(240);
                    ArmMotor.setPower(0.4);
                    ArmOut = true;
                } else if (gamepad2.a) {
                    ArmMotor.setTargetPosition(195);
                    ArmMotor.setPower(0.45);
                    ArmOut = true;
                }
            }
            //LiftPos = ArmMotor.getCurrentPosition();

            //grabber
            if (gamepad2.left_bumper) {
                GrabServo.setPosition(0.0);
            } else if (gamepad2.right_bumper) {
                GrabServo.setPosition(0.85);
            }
            //if (gamepad1.left_bumper == true) {
            //    LiftMotor.setPower(0.25);
            //}
            //else if (gamepad1.right_bumper == true) {
            //    LiftMotor.setPower(-0.25);
            //} else {
            //    LiftMotor.setPower(0);
            //}
            //LiftPos = LiftMotor.getCurrentPosition();

            if (gamepad1.a == true) {
                gearshift = 0.25;
                turnspeed = 0.4;
            } else if (gamepad1.b == true) {
                gearshift = 0.5;
                turnspeed = 0.65;
            } else if (gamepad1.x == true) {
                gearshift = 0.75;
                turnspeed = 0.8;
            } else if (gamepad1.y == true) {
                gearshift = 0.9;
                turnspeed = 1.0;
            }
            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            /*
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("lift possision: ", "%4.2f", LiftPos);
            */
            telemetry.update();
        }
    }
}