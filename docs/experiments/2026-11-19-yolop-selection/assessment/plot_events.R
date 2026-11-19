library(jsonlite)
records <- fromJSON("field-measurements.json", simplifyVector = FALSE)
by_name <- function(prefix) {
  matches <- vapply(records, function(item) startsWith(item$session, prefix), logical(1))
  records[[which(matches)]]$result
}
series <- function(rows, index, type) {
  vapply(rows, function(row) row[[index]], type)
}
red <- by_name("rosbag2_2026_11_19-16_55_15")$plot
fault <- by_name("concept_20261119_170817")$invalid_gate_window$plot
ink <- "#202e39"
grid <- "#dce2e5"
left <- "#116658"
right <- "#3b66a0"
danger <- "#a73b2e"
pale <- "#f6dfd9"
png("field-events.png", width = 1450, height = 870, res = 140, type = "cairo", bg = "white")
par(mfrow = c(2, 1), mar = c(3.5, 4.8, 3.3, 1.1), family = "Noto Sans CJK SC",
    mgp = c(2.8, 0.7, 0), bty = "n", col.axis = ink, col.lab = ink, fg = ink)
t <- series(red$wheel, 1, numeric(1))
r <- series(red$wheel, 2, numeric(1))
l <- series(red$wheel, 3, numeric(1))
gtime <- series(red$gate, 1, numeric(1))
hold <- series(red$gate, 2, character(1)) == "semantic_hold:red_wait_first_lap"
plot(range(t), c(-10, 200), type = "n", axes = FALSE, xlab = "", ylab = "轮速反馈（RPM）", xaxs = "i")
for (i in which(hold)) {
  if (i < length(gtime)) rect(gtime[i], -10, gtime[i + 1], 200, col = pale, border = NA)
}
abline(h = seq(0, 200, 50), col = grid)
axis(1, at = seq(-1, 2.5, 0.5), tck = -0.025)
axis(2, at = seq(0, 200, 50), las = 1, tck = -0.018)
lines(t, r, col = right, lwd = 2)
lines(t, l, col = left, lwd = 2)
title(main = "MapAssist  |  首圈红灯等待期间的实测轮速", adj = 0, col.main = ink, cex.main = 1.15)
mtext("相对首次红灯门控的时间（秒）    0 = 16:57:59.597 JST", 1, line = 2.3, cex = 0.81)
legend("topright", c("右轮", "左轮", "旧门控红灯等待"), col = c(right, left, pale),
       lwd = c(2.5, 2.5, 8), horiz = TRUE, bty = "n", cex = 0.75, bg = "white")
mark <- which(r == 0 & l == 0 & t >= 0)[1]
points(t[mark], 0, pch = 21, bg = "white", col = ink, lwd = 1.25)
segments(t[mark], 4, t[mark], 34, col = ink, lwd = 0.7)
text(t[mark], 42, "两轮首次归零", pos = 4, cex = 0.74, col = ink)
ts <- series(fault$state, 1, numeric(1))
valid <- series(fault$state, 2, logical(1))
tg <- series(fault$gate, 1, numeric(1))
auto <- series(fault$gate, 2, character(1)) == "auto"
plot(c(-0.9, 5.4), c(-0.4, 1.42), type = "n", axes = FALSE, xlab = "", ylab = "", xaxs = "i")
abline(v = 0:5, col = grid)
axis(1, at = 0:5, tck = -0.025)
axis(2, at = c(0, 1), labels = c("门控模式", "车况有效性"), las = 1, tick = FALSE, line = -0.30)
points(ts, rep(1, length(ts)), pch = 15, cex = 0.49, col = ifelse(valid, left, danger))
points(tg, rep(0, length(tg)), pch = 15, cex = 0.64, col = ifelse(auto, left, danger))
text(2.7, 0.24, "从首次暂停到最后一条门控记录共 5.317 秒", cex = 0.80, col = danger)
text(2.82, 1.2, "车况恢复有效后，门控仍处于暂停", cex = 0.78, col = ink)
title(main = "Concept  |  17:08 场次的车况与门控", adj = 0, col.main = ink, cex.main = 1.15)
mtext("相对首次车况暂停的时间（秒）    0 = 17:08:40.142 JST", 1, line = 2.3, cex = 0.81)
legend("bottomright", c("有效 / 自动", "无效 / 暂停"), text.col = ink, col = c(left, danger),
       pch = 15, horiz = TRUE, bty = "n", cex = 0.76, inset = c(0.005, -0.035))
dev.off()
