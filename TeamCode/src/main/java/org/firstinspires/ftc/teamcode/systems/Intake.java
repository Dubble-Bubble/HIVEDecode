package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Intake {

    private DcMotor intake;
    private Servo flapL, flapR;

    public static double shotDelay = 1.2, shotTransferAllowance = 0.4;
    public static double flapActuationTime = 0.8;

    private boolean intakeActivityFlag = false;
    private boolean shootingFlag = false;

    public static double flapUp = 0.17, flapDown = 0.2;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.dcMotor.get("intake");
        flapL = hardwareMap.servo.get("lFlap");
        flapR = hardwareMap.servo.get("rFlap");
        flapR.setDirection(Servo.Direction.REVERSE);
    }

    public void setActive(boolean flag) {
        intakeActivityFlag = flag;
    }

    public void setShootingMode(boolean flag) {
        shootingFlag = flag;
    }

    ElapsedTime shotTimer = new ElapsedTime();
    private boolean wasShooting = false;

    int shotCount = 0;

    public void update() {
        if (!shootingFlag && intakeActivityFlag) {
            wasShooting = false;
            intake.setPower(1);
            setFlap(flapDown);
        } else if (!shootingFlag && !intakeActivityFlag) {
            wasShooting = false;
            setFlap(flapDown);
            intake.setPower(0);
        } else if (shootingFlag && !intakeActivityFlag){
            if (!wasShooting) {
                wasShooting = true;
                shotTimer.reset();
                setFlap(flapUp);
                shotCount = 0;
                shotDelay = 1;
            }

            double poll = shotTimer.seconds();

            if (poll < shotTransferAllowance) {
                intake.setPower(1);
            } else if (poll > shotTransferAllowance && poll < shotDelay) {
                intake.setPower(0);
            } else {
                shotTimer.reset();
                shotCount += 1;
            }
        }
    }

    public void setIntake(double power) {
        intake.setPower(power);
    }

    public void setFlap(double pos) {
        flapR.setPosition(pos);
        flapL.setPosition(pos);
    }

}
