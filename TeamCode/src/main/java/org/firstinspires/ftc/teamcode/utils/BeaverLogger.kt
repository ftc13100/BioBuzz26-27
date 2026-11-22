package org.firstinspires.ftc.teamcode.utils

import java.io.BufferedWriter
import java.io.File
import java.io.FileWriter

class BeaverLogger(path: String = "/sdcard/FIRST/log.csv") {
    private val writer: BufferedWriter
    private var columns: List<String>? = null

    init {
        val file = File(path)
        file.parentFile?.mkdirs()
        writer = BufferedWriter(FileWriter(file))
    }

    fun log(vararg fields: Pair<String, Any?>) {
        log(linkedMapOf(*fields))
    }

    fun log(fields: Map<String, Any?>) {
        val cols = columns ?: fields.keys.toList().also {
            columns = it
            writer.write(it.joinToString(","))
            writer.newLine()
        }
        writer.write(cols.joinToString(",") { fields[it]?.toString() ?: "" })
        writer.newLine()
    }

    fun close() {
        writer.flush()
        writer.close()
    }
}