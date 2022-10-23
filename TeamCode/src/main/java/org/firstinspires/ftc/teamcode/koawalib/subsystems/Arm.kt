package org.firstinspires.ftc.teamcode.koawalib.subsystems

import com.asiankoala.koawalib.hardware.motor.KMotor
import com.asiankoala.koawalib.subsystem.Subsystem

class Arm(val motor: KMotor) : Subsystem() {
    companion object {
        val ticksPerUnit: Double = TODO()
        val homePos : Double = TODO()
        val groundPos : Double = TODO()
        val lowPos : Double = TODO()
        val midPos : Double = TODO()
        val highPos : Double = TODO()
    }
    fun setPos(pos: Double) {
        motor.setProfileTarget(pos)
    }
}