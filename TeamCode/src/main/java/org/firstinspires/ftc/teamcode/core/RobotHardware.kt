package org.firstinspires.ftc.teamcode.core

enum class RobotHardware(val deviceName: String) {
    M_WHEEL_FR("motor0"),
    M_WHEEL_BR("motor1"),
    M_WHEEL_FL("motor2"),
    M_WHEEL_BL("motor3"),
    M_SHOOTER("motor5"),
    M_SPINDEXER("motor6"),
    M_INTAKE("motor7"),

    S_TURRET1("servo1"),
    S_TURRET2("servo2"),
    S_TURRET_HOOD("servo3"),

    A_TURRET("analog0"),
    A_SPINDEXER("analog4"),

    D_TURRET("digital0"),

    I2C_PINPOINT("i2c1"),
    I2C_LED("i2c2"),
    I2C_COLOR1("i2c4"),
    I2C_COLOR2("i2c5"),
    I2C_COLOR3("i2c6"),
}