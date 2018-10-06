package org.firstinspires.ftc.team5391;

public class CheesyDrive {

    double mQuickStopAccumulator;
    public static final double kThrottleDeadband = 0.02;
    private static final double kWheelDeadband = 0.02;
    private static final double kTurnSensitivity = 1.0;
    private DriveSignal mSignal = new DriveSignal(0, 0);

    public DriveSignal cheesyDrive(double throttle, double wheel, boolean isQuickTurn) {

        wheel = handleDeadband(wheel, kWheelDeadband);
        throttle = handleDeadband(throttle, kThrottleDeadband);

        double overPower;

        double angularPower;

        if (isQuickTurn) {
            if (Math.abs(throttle) < 0.2) {
                double alpha = 0.1;
                mQuickStopAccumulator = (1 - alpha) * mQuickStopAccumulator + alpha * Util.limit(wheel, 1.0) * 2;
            }
            overPower = 1.0;
            angularPower = wheel;
        } else {
            overPower = 0.0;
            angularPower = Math.abs(throttle) * wheel * kTurnSensitivity - mQuickStopAccumulator;
            if (mQuickStopAccumulator > 1) {
                mQuickStopAccumulator -= 1;
            } else if (mQuickStopAccumulator < -1) {
                mQuickStopAccumulator += 1;
            } else {
                mQuickStopAccumulator = 0.0;
            }
        }

        double rightPwm = throttle - angularPower;
        double leftPwm = throttle + angularPower;
        if (leftPwm > 1.0) {
            rightPwm -= overPower * (leftPwm - 1.0);
            leftPwm = 1.0;
        } else if (rightPwm > 1.0) {
            leftPwm -= overPower * (rightPwm - 1.0);
            rightPwm = 1.0;
        } else if (leftPwm < -1.0) {
            rightPwm += overPower * (-1.0 - leftPwm);
            leftPwm = -1.0;
        } else if (rightPwm < -1.0) {
            leftPwm += overPower * (-1.0 - rightPwm);
            rightPwm = -1.0;
        }
        mSignal.rightMotor = rightPwm;
        mSignal.leftMotor = leftPwm;
        return mSignal;
    }

    public double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }
}
