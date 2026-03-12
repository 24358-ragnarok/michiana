package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FieldRenderer {
    private final Telemetry.Item fieldItem;
    private final int width = 73;
    private final int height = 74;
    private final boolean[][] pixels = new boolean[height][width];
    private final String[][] cellColors = new String[(height + 3) / 4][(width + 1) / 2];

    private boolean[][] snapshotPixels;
    private String[][] snapshotColors;

    private final double scaleX = width / 144.0;
    private final double scaleY = height / 144.0;

    public FieldRenderer(Telemetry telemetry) {
        this.fieldItem = telemetry.addData("", "");
        drawStaticField();
        snapshot();
    }

    public void render(double x, double y, double heading, String color) {
        restore();
        drawRobot(x, y, heading, color);
        fieldItem.setValue("<br/><small><pre style='line-height:1; letter-spacing:0;'>" + buildHtml() + "</pre></small>");
    }

    private void drawStaticField() {
        int w = width - 1;
        int h = height - 1;
        String grid = "#444444";

        drawRect(0, 0, w, h, "#666666");
        // Grid lines
        for (int i = 1; i < 6; i++) {
            int px = (int) ((i / 6.0) * w);
            int py = (int) ((i / 6.0) * h);
            drawLine(px, 0, px, h, grid);
            drawLine(0, py, w, py, grid);
        }
    }

    private void drawRobot(double xIn, double yIn, double heading, String color) {
        int px = (int) (xIn * scaleX);
        int py = (height - 1) - (int) (yIn * scaleY);
        int r = (int) (9 * scaleX); // 18" robot radius

        drawCircle(px, py, r, color);
        int lx = (int) (px + Math.cos(heading) * r);
        int ly = (int) (py - Math.sin(heading) * r);
        drawLine(px, py, lx, ly, color);
    }

    private void setPixel(int x, int y, String color) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        pixels[y][x] = true;
        cellColors[y / 4][x / 2] = color;
    }

    private void drawLine(int x0, int y0, int x1, int y1, String color) {
        int dx = Math.abs(x1 - x0), dy = Math.abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        while (true) {
            setPixel(x0, y0, color);
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y0 += sy;
            }
        }
    }

    private void drawRect(int x, int y, int w, int h, String color) {
        drawLine(x, y, x + w, y, color);
        drawLine(x, y + h, x + w, y + h, color);
        drawLine(x, y, x, y + h, color);
        drawLine(x + w, y, x + w, y + h, color);
    }

    private void drawCircle(int cx, int cy, int r, String color) {
        int x = r, y = 0, err = 0;
        while (x >= y) {
            setPixel(cx + x, cy + y, color);
            setPixel(cx + y, cy + x, color);
            setPixel(cx - y, cy + x, color);
            setPixel(cx - x, cy + y, color);
            setPixel(cx - x, cy - y, color);
            setPixel(cx - y, cy - x, color);
            setPixel(cx + y, cy - x, color);
            setPixel(cx + x, cy - y, color);
            if (err <= 0) {
                y++;
                err += 2 * y + 1;
            }
            if (err > 0) {
                x--;
                err -= 2 * x + 1;
            }
        }
    }

    private void snapshot() {
        snapshotPixels = new boolean[height][width];
        snapshotColors = new String[cellColors.length][cellColors[0].length];
        for (int i = 0; i < height; i++) snapshotPixels[i] = pixels[i].clone();
        for (int i = 0; i < cellColors.length; i++) snapshotColors[i] = cellColors[i].clone();
    }

    private void restore() {
        for (int i = 0; i < height; i++)
            System.arraycopy(snapshotPixels[i], 0, pixels[i], 0, width);
        for (int i = 0; i < cellColors.length; i++)
            System.arraycopy(snapshotColors[i], 0, cellColors[i], 0, cellColors[0].length);
    }

    private String buildHtml() {
        StringBuilder sb = new StringBuilder();
        for (int y = 0; y < height; y += 4) {
            String lastCol = null;
            for (int x = 0; x < width; x += 2) {
                String color = cellColors[y / 4][x / 2];
                if (color != null && !color.equals(lastCol)) {
                    if (lastCol != null) sb.append("</font>");
                    sb.append("<font color='").append(color).append("'>");
                    lastCol = color;
                } else if (color == null && lastCol != null) {
                    sb.append("</font>");
                    lastCol = null;
                }
                sb.append(getBraille(x, y));
            }
            if (lastCol != null) sb.append("</font>");
            sb.append("\n");
        }
        return sb.toString();
    }

    private char getBraille(int x, int y) {
        int code = 0;
        if (isTrue(x, y)) code |= 1;
        if (isTrue(x, y + 1)) code |= 2;
        if (isTrue(x, y + 2)) code |= 4;
        if (isTrue(x + 1, y)) code |= 8;
        if (isTrue(x + 1, y + 1)) code |= 16;
        if (isTrue(x + 1, y + 2)) code |= 32;
        if (isTrue(x, y + 3)) code |= 64;
        if (isTrue(x + 1, y + 3)) code |= 128;
        return (char) (0x2800 + code);
    }

    private boolean isTrue(int x, int y) {
        return x >= 0 && x < width && y >= 0 && y < height && pixels[y][x];
    }
}