package org.firstinspires.ftc.teamcode.core

enum class RobotHardware(val deviceName: String) {
    M_WHEEL_FR("frontRight"),
    M_WHEEL_BR("backRight"),
    M_WHEEL_FL("frontLeft"),
    M_WHEEL_BL("backLeft"),
    M_SHOOTER("shooter"),
    M_SPINDEXER("spindexer"),
    M_INTAKE("intake"),

    S_TURRET1("turret1"),
    S_TURRET2("turret2"),
    S_TURRET_HOOD("angle"),

    A_TURRET("analog0"),
    A_SPINDEXER("analogS"),

    Q_TURRET("frontRight"),

    I2C_OCTOQUAD("octoquad"),

    CAM_TURRET("turretArducam"),
    LL_INTAKE("turretIntake")
}
