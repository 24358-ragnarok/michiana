package org.firstinspires.ftc.teamcode.util.telemetry;

/**
 * A builder class for constructing formatted telemetry lines.
 * <p>
 * Allows chaining of text segments with different styles (bold, color, etc.)
 * and provides output in both HTML (for Driver Station) and plain text (for Panels).
 */
public class LogLine {
    private final StringBuilder builder = new StringBuilder();

    /**
     * Appends plain text to the line.
     */
    public LogLine append(String text) {
        builder.append(text);
        return this;
    }

    /**
     * Appends colored text to the line.
     *
     * @param text     The text to append.
     * @param hexColor The hex color code (e.g., "#FF0000").
     */
    public LogLine appendColor(String text, String hexColor) {
        builder.append(TextFormat.color(text, hexColor));
        return this;
    }

    /**
     * Appends bold text to the line.
     */
    public LogLine appendBold(String text) {
        builder.append(TextFormat.bold(text));
        return this;
    }

    /**
     * Appends text styled as a success message (green).
     */
    public LogLine appendSuccess(String text) {
        builder.append(TextFormat.success(text));
        return this;
    }

    /**
     * Appends text styled as an error message (red).
     */
    public LogLine appendError(String text) {
        builder.append(TextFormat.error(text));
        return this;
    }

    /**
     * Returns the raw HTML string for the Driver Station.
     */
    public String getHtml() {
        return builder.toString();
    }

    /**
     * Returns the stripped plaintext string for Panels compatibility.
     */
    public String getPlainText() {
        return TextFormat.stripHtml(builder.toString());
    }

    @Override
    public String toString() {
        return getHtml();
    }
}
