package org.firstinspires.ftc.teamcode.kotlin_mirror.util.telemetry

/**
 * Utility class for formatting text with HTML tags for the Driver Station telemetry.
 *
 * Provides methods for coloring, sizing, and styling text, as well as semantic presets
 * for headers, success messages, errors, and warnings.
 */
object TextFormat {

    // --- Base HTML Wrappers ---

    @JvmStatic
    fun color(text: String, hexColor: String): String {
        return "<font color='$hexColor'>$text</font>"
    }

    @JvmStatic
    fun bold(text: String): String {
        return "<strong>$text</strong>"
    }

    @JvmStatic
    fun italic(text: String): String {
        return "<em>$text</em>"
    }

    @JvmStatic
    fun big(text: String): String {
        return "<big>$text</big>"
    }

    @JvmStatic
    fun small(text: String): String {
        return "<small>$text</small>"
    }

    // --- Semantic Presets ---

    @JvmStatic
    fun header(text: String): String {
        return big(bold(color(text, "#448aff"))) // FTC Blue
    }

    @JvmStatic
    fun subheader(text: String): String {
        return bold(color(text, "#448aff")) // FTC Blue
    }

    @JvmStatic
    fun success(text: String): String {
        return bold(color(text, "#00ff88")) // Bright Green
    }

    @JvmStatic
    fun error(text: String): String {
        return bold(color(text, "#ef5350")) // Alert Red
    }

    @JvmStatic
    fun warn(text: String): String {
        return color(text, "#ffeb3b") // Yellow
    }

    // --- Utilities ---

    /**
     * Strips all HTML tags from a string.
     *
     * Required for compatibility with telemetry sinks that do not support HTML,
     * such as the FTControl Panels log.
     *
     * @param html The HTML string to strip.
     * @return The plain text string.
     */
    @JvmStatic
    fun stripHtml(html: String?): String? {
        return html?.replace(Regex("<[^>]*>"), "")
    }
}
