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

    D_TURRET("digital0"),

    I2C_PINPOINT("pinpoint"),
    I2C_LED("i2c2"),
    I2C_COLOR0("cs0"),
    I2C_COLOR1("cs1"),
    I2C_COLOR2("cs2"),
}
