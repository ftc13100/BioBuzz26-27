package org.firstinspires.ftc.teamcode.core

enum class RobotHardware(val deviceName: String) {
    M_WHEEL_FR("frontRight"),
    M_WHEEL_BR("backRight"),
    M_WHEEL_FL("frontLeft"),
    M_WHEEL_BL("backLeft"),
    M_SHOOTER("shooter"),
    M_TRANSFER("transfer"),
    M_INTAKE("intake"),

    S_TURRET("turret"),
    S_HOOD("hood"),
    S_GATE("gate"),
    S_RAMP("ramp"),

    I2C_OCTOQUAD("octoquad"),

    CAM_TURRET("arducam"),
    CAM_LL("limelight")
}

enum class OctoquadPorts(val channel: Int) {
    Q_SHOOTER(3),
    Q_TRANSFER(6),
    Q_TURRET(7)
}