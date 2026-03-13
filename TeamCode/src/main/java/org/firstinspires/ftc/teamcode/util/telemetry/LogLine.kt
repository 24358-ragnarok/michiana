package org.firstinspires.ftc.teamcode.util.telemetry

/**
 * A builder class for constructing formatted telemetry lines.
 *
 * Allows chaining of text segments with different styles (bold, color, etc.)
 * and provides output in both HTML (for Driver Station) and plain text (for Panels).
 */
class LogLine {
    private val builder = StringBuilder()

    /**
     * Appends plain text to the line.
     */
    fun append(text: String): LogLine {
        builder.append(text)
        return this
    }

    /**
     * Appends colored text to the line.
     *
     * @param text     The text to append.
     * @param hexColor The hex color code (e.g., "#FF0000").
     */
    fun appendColor(text: String, hexColor: String): LogLine {
        builder.append(TextFormat.color(text, hexColor))
        return this
    }

    /**
     * Appends bold text to the line.
     */
    fun appendBold(text: String): LogLine {
        builder.append(TextFormat.bold(text))
        return this
    }

    /**
     * Appends text styled as a success message (green).
     */
    fun appendSuccess(text: String): LogLine {
        builder.append(TextFormat.success(text))
        return this
    }

    /**
     * Appends text styled as an error message (red).
     */
    fun appendError(text: String): LogLine {
        builder.append(TextFormat.error(text))
        return this
    }

    /**
     * Returns the raw HTML string for the Driver Station.
     */
    val html: String
        get() = builder.toString()

    /**
     * Returns the stripped plaintext string for Panels compatibility.
     */
    val plainText: String?
        get() = TextFormat.stripHtml(builder.toString())

    override fun toString(): String {
        return html
    }
}
