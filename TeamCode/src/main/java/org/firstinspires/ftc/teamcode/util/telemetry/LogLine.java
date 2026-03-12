package org.firstinspires.ftc.teamcode.util.telemetry;

public class LogLine {
    private final StringBuilder builder = new StringBuilder();

    public LogLine append(String text) {
        builder.append(text);
        return this;
    }

    public LogLine appendColor(String text, String hexColor) {
        builder.append(TextFormat.color(text, hexColor));
        return this;
    }

    public LogLine appendBold(String text) {
        builder.append(TextFormat.bold(text));
        return this;
    }

    public LogLine appendSuccess(String text) {
        builder.append(TextFormat.success(text));
        return this;
    }

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
     * Returns the stripped plaintext string for Panels.
     */
    public String getPlainText() {
        return TextFormat.stripHtml(builder.toString());
    }

    @Override
    public String toString() {
        return getHtml();
    }
}