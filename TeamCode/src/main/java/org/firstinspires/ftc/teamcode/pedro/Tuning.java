package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.OctoQuadTuner;

public class Tuning {

    @Tuner
    public static Procedure octoquadTuner() {
        return new OctoQuadTuner();
    }
}
