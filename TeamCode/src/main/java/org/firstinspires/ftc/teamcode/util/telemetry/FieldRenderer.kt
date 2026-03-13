package org.firstinspires.ftc.teamcode.util.telemetry

import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

/**
 * Renders a visual representation of the field and robot on the Driver Station telemetry.
 *
 * Uses Braille Unicode characters to create a low-resolution pixel grid that can be
 * displayed in the text-based telemetry log.
 */
class FieldRenderer(telemetry: Telemetry) {
    private val fieldItem: Telemetry.Item = telemetry.addData("", "")
    private val width = 73
    private val height = 74
    private val pixels = Array(height) { BooleanArray(width) }
    private val cellColors = Array((height + 3) / 4) { arrayOfNulls<String>((width + 1) / 2) }

    private lateinit var snapshotPixels: Array<BooleanArray>
    private lateinit var snapshotColors: Array<Array<String?>>

    private val scaleX = width / 144.0
    private val scaleY = height / 144.0

    init {
        drawStaticField()
        snapshot()
    }

    /**
     * Renders the field with the robot at the specified position.
     *
     * @param x       The robot's X coordinate (inches).
     * @param y       The robot's Y coordinate (inches).
     * @param heading The robot's heading (radians).
     * @param color   The color of the robot indicator.
     */
    fun render(x: Double, y: Double, heading: Double, color: String) {
        restore()
        drawRobot(x, y, heading, color)
        fieldItem.setValue("<br/><small><pre style='line-height:1; letter-spacing:0;'>" + buildHtml() + "</pre></small>")
    }

    private fun drawStaticField() {
        val w = width - 1
        val h = height - 1
        val grid = "#444444"

        drawRect(0, 0, w, h, "#666666")
        // Grid lines
        for (i in 1..5) {
            val px = ((i / 6.0) * w).toInt()
            val py = ((i / 6.0) * h).toInt()
            drawLine(px, 0, px, h, grid)
            drawLine(0, py, w, py, grid)
        }
    }

    private fun drawRobot(xIn: Double, yIn: Double, heading: Double, color: String) {
        val px = (xIn * scaleX).toInt()
        val py = (height - 1) - (yIn * scaleY).toInt()
        val r = (9 * scaleX).toInt() // 18" robot radius

        drawCircle(px, py, r, color)
        val lx = (px + cos(heading) * r).toInt()
        val ly = (py - sin(heading) * r).toInt()
        drawLine(px, py, lx, ly, color)
    }

    private fun setPixel(x: Int, y: Int, color: String) {
        if (x < 0 || x >= width || y < 0 || y >= height) return
        pixels[y][x] = true
        cellColors[y / 4][x / 2] = color
    }

    private fun drawLine(x0: Int, y0: Int, x1: Int, y1: Int, color: String) {
        var x0Var = x0
        var y0Var = y0
        val dx = abs(x1 - x0Var)
        val dy = abs(y1 - y0Var)
        val sx = if (x0Var < x1) 1 else -1
        val sy = if (y0Var < y1) 1 else -1
        var err = dx - dy
        while (true) {
            setPixel(x0Var, y0Var, color)
            if (x0Var == x1 && y0Var == y1) break
            val e2 = 2 * err
            if (e2 > -dy) {
                err -= dy
                x0Var += sx
            }
            if (e2 < dx) {
                err += dx
                y0Var += sy
            }
        }
    }

    private fun drawRect(x: Int, y: Int, w: Int, h: Int, color: String) {
        drawLine(x, y, x + w, y, color)
        drawLine(x, y + h, x + w, y + h, color)
        drawLine(x, y, x, y + h, color)
        drawLine(x + w, y, x + w, y + h, color)
    }

    private fun drawCircle(cx: Int, cy: Int, r: Int, color: String) {
        var x = r
        var y = 0
        var err = 0
        while (x >= y) {
            setPixel(cx + x, cy + y, color)
            setPixel(cx + y, cy + x, color)
            setPixel(cx - y, cy + x, color)
            setPixel(cx - x, cy + y, color)
            setPixel(cx - x, cy - y, color)
            setPixel(cx - y, cy - x, color)
            setPixel(cx + y, cy - x, color)
            setPixel(cx + x, cy - y, color)
            if (err <= 0) {
                y++
                err += 2 * y + 1
            }
            if (err > 0) {
                x--
                err -= 2 * x + 1
            }
        }
    }

    private fun snapshot() {
        snapshotPixels = Array(height) { i -> pixels[i].clone() }
        snapshotColors = Array(cellColors.size) { i -> cellColors[i].clone() }
    }

    private fun restore() {
        for (i in 0 until height)
            System.arraycopy(snapshotPixels[i], 0, pixels[i], 0, width)
        for (i in cellColors.indices)
            System.arraycopy(snapshotColors[i], 0, cellColors[i], 0, cellColors[0].size)
    }

    private fun buildHtml(): String {
        val sb = StringBuilder()
        for (y in 0 until height step 4) {
            var lastCol: String? = null
            for (x in 0 until width step 2) {
                val color = cellColors[y / 4][x / 2]
                if (color != null && color != lastCol) {
                    if (lastCol != null) sb.append("</font>")
                    sb.append("<font color='").append(color).append("'>")
                    lastCol = color
                } else if (color == null && lastCol != null) {
                    sb.append("</font>")
                    lastCol = null
                }
                sb.append(getBraille(x, y))
            }
            if (lastCol != null) sb.append("</font>")
            sb.append("\n")
        }
        return sb.toString()
    }

    private fun getBraille(x: Int, y: Int): Char {
        var code = 0
        if (isTrue(x, y)) code = code or 1
        if (isTrue(x, y + 1)) code = code or 2
        if (isTrue(x, y + 2)) code = code or 4
        if (isTrue(x + 1, y)) code = code or 8
        if (isTrue(x + 1, y + 1)) code = code or 16
        if (isTrue(x + 1, y + 2)) code = code or 32
        if (isTrue(x, y + 3)) code = code or 64
        if (isTrue(x + 1, y + 3)) code = code or 128
        return (0x2800 + code).toChar()
    }

    private fun isTrue(x: Int, y: Int): Boolean {
        return x >= 0 && x < width && y >= 0 && y < height && pixels[y][x]
    }
}
