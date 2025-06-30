package org.firstinspires.ftc.teamcode.autonoom;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kp, ki, kd;
    private double integralSum = 0;
    private double lastError = 0;
    // Elapsed timer class from SDK, please use it, it's epic
    ElapsedTime timer = new ElapsedTime();

    public PIDController(double Kp, double Ki, double Kd) {
        kp = Kp;
        ki = Ki;
        kd = Kd;
    }

    public void setParams(double Kp, double Ki, double Kd) {
        kp = Kp;
        ki = Ki;
        kd = Kd;
    }

    /**
     * update the PID controller output
     * @param target where we would like to be, also called the reference
     * @param state where we currently are, I.E. motor position
     * @return the command to our motor, I.E. motor power
     */
    public double update(double target, double state) {
        // PID logic and then return the output
        // obtain the encoder position
        //encoderPosition = armMotor.getPosition();
        double error = target - state;

        double derivative = (error - lastError) / timer.seconds();

        integralSum = integralSum + (error * timer.seconds());

        double out = (kp * error) + (ki * integralSum) + (kd * derivative);

        lastError = error;

        timer.reset();
        return out;

    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }
}


