package org.firstinspires.ftc.teamcode.arm;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.util.hardware.AxonEncoder;
import org.firstinspires.ftc.teamcode.util.hardware.SuccessCRServo;

import java.util.ArrayList;
import java.util.List;

public class Pivot {
    private final SuccessCRServo fr;
    private final SuccessCRServo fl;
    private final SuccessCRServo bl;
    private final SuccessCRServo br;
    private final AxonEncoder encoder;

    private final List<SuccessCRServo> servos = new ArrayList<>();

    public Pivot() {
        fr = new SuccessCRServo(RobotMap.getInstance().PIVOT_FR);
        fl = new SuccessCRServo(RobotMap.getInstance().PIVOT_FL);
        bl = new SuccessCRServo(RobotMap.getInstance().PIVOT_BL);
        br = new SuccessCRServo(RobotMap.getInstance().PIVOT_BR);
        encoder = new AxonEncoder(RobotMap.getInstance().PIVOT_ENCODER);

        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        servos.add(fr);
        servos.add(fl);
        servos.add(bl);
        servos.add(br);
    }

    public void setPower(double power) {
        for (SuccessCRServo s : servos) {
            s.setPower(power);
        }
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public double getPower() {
        return fr.getPower();
    }
}
