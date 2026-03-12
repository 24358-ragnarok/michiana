package org.firstinspires.ftc.teamcode.util.telemetry;

public class TextFormat {

    // --- Base HTML Wrappers ---
    public static String color(String text, String hexColor) {
        return "<font color='" + hexColor + "'>" + text + "</font>";
    }

    public static String bold(String text) {
        return "<strong>" + text + "</strong>";
    }

    public static String italic(String text) {
        return "<em>" + text + "</em>";
    }

    public static String big(String text) {
        return "<big>" + text + "</big>";
    }

    public static String small(String text) {
        return "<small>" + text + "</small>";
    }

    // --- Semantic Presets ---
    public static String header(String text) {
        return big(bold(color(text, "#448aff"))); // FTC Blue
    }

    public static String subheader(String text) {
        return bold(color(text, "#448aff")); // FTC Blue
    }

    public static String success(String text) {
        return bold(color(text, "#00ff88")); // Bright Green
    }

    public static String error(String text) {
        return bold(color(text, "#ef5350")); // Alert Red
    }

    public static String warn(String text) {
        return color(text, "#ffeb3b"); // Yellow
    }

    // --- Utilities ---

    /**
     * Strips all HTML tags from a string. Required for Panels compatibility.
     */
    public static String stripHtml(String html) {
        if (html == null) return null;
        return html.replaceAll("<[^>]*>", "");
    }
}
