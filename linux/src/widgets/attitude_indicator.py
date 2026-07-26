# widgets/attitude_indicator.py
"""
Attitude Indicator - Custom QWidget
Shows aircraft pitch and roll
"""

from PyQt5.QtWidgets import QWidget, QSizePolicy
from PyQt5.QtCore import Qt, QRectF, QPointF, QSize
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QFont, QLinearGradient


class AttitudeIndicator(QWidget):
    """Attitude indicator showing pitch and roll with execution time bar"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.pitch = 0.0
        self.roll = 0.0
        self.heading = 0.0
        self.exec_current = 0.0
        self.exec_peak = 0.0
        self.setMinimumSize(150, 150)
        
        # Colors
        self.sky_top = QColor(70, 130, 180)
        self.sky_bottom = QColor(135, 206, 235)
        self.ground_top = QColor(139, 69, 19)
        self.ground_bottom = QColor(34, 139, 34)
        self.text_color = QColor(255, 255, 255)
        self.pointer_color = QColor(255, 0, 0)
        self.background_color = QColor(30, 30, 30)
        self.bar_peak_color = QColor(180, 0, 0)
        self.bar_current_color = QColor(0, 120, 255)
    
    def set_attitude(self, pitch: float, roll: float, heading: float = 0.0):
        """Set pitch (degrees), roll (degrees), heading (degrees)"""
        self.pitch = max(-90, min(90, pitch))
        self.roll = roll
        self.heading = heading
        self.update()
    
    def set_exec_time(self, current_percent: float, peak_percent: float):
        """Set CPU execution time for bar graph (0-100) — no longer drawn here."""
        pass
    
    def paintEvent(self, event):
        """Draw the attitude indicator"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        cx = self.width() // 2
        cy = self.height() // 2
        radius = min(cx, cy) - 10
        
        # Background
        painter.setBrush(QBrush(self.background_color))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(QPointF(cx, cy), radius, radius)
        
        painter.setClipRect(QRectF(cx - radius, cy - radius, radius * 2, radius * 2))
        painter.translate(cx, cy)
        painter.rotate(-self.roll)
        
        pitch_scale = radius / 100.0
        pitch_offset = int(-self.pitch * pitch_scale)
        r = int(radius)
        
        # Sky
        sky_grad = QLinearGradient(0, -r, 0, r)
        sky_grad.setColorAt(0, self.sky_top)
        sky_grad.setColorAt(1, self.sky_bottom)
        painter.setBrush(QBrush(sky_grad))
        painter.setPen(Qt.NoPen)
        painter.drawRect(-r, -r, r * 2, r + pitch_offset)
        
        # Ground
        ground_grad = QLinearGradient(0, -r, 0, r)
        ground_grad.setColorAt(0, self.ground_top)
        ground_grad.setColorAt(1, self.ground_bottom)
        painter.setBrush(QBrush(ground_grad))
        painter.drawRect(-r, pitch_offset, r * 2, r * 2)
        
        # Horizon line
        painter.setPen(QPen(Qt.white, 2))
        painter.drawLine(-r, 0, r, 0)
        
        # Pitch ladder
        painter.setPen(QPen(self.text_color, 1))
        font = QFont("Arial", 8)
        painter.setFont(font)
        
        for pitch_deg in range(-90, 91, 10):
            y = int(pitch_offset + pitch_deg * pitch_scale)
            if abs(y) > r * 0.85:
                continue
            
            line_len = int(r * 0.4) if pitch_deg % 10 == 0 else int(r * 0.15)
            if pitch_deg % 10 == 0 and pitch_deg != 0:
                label = str(abs(pitch_deg))
                painter.drawText(-line_len - 15, y + 4, label)
                painter.drawText(line_len + 5, y + 4, label)
            
            painter.drawLine(-line_len, y, line_len, y)
        
        painter.rotate(self.roll)
        
        # Roll pointer
        painter.setPen(QPen(self.pointer_color, 2))
        painter.setBrush(QBrush(self.pointer_color))
        pointer = [
            QPointF(0, -r + 10),
            QPointF(-8, -r + 25),
            QPointF(8, -r + 25)
        ]
        painter.drawPolygon(*pointer)
        
        # Aircraft symbol
        painter.setPen(QPen(Qt.black, 2))
        painter.setBrush(QBrush(Qt.white))
        painter.drawEllipse(QPointF(0, 0), 8, 8)
        
        painter.setPen(QPen(Qt.black, 1))
        painter.setBrush(Qt.NoBrush)
        painter.drawLine(-15, 0, -5, 0)
        painter.drawLine(5, 0, 15, 0)
        painter.drawLine(0, -15, 0, -5)
        painter.drawLine(0, 5, 0, 15)
        
        # Border
        painter.setClipRect(QRectF(-r, -r, r * 2, r * 2))
        painter.setPen(QPen(Qt.white, 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(QPointF(0, 0), r, r)
        
        # Heading text at bottom
        painter.setPen(QPen(self.text_color, 1))
        painter.setBrush(Qt.NoBrush)
        heading_font = QFont("Arial", 14, QFont.Bold)
        painter.setFont(heading_font)
        heading_text = f"{int(self.heading % 360)}°"
        heading_rect = painter.fontMetrics().boundingRect(heading_text)
        painter.drawText(-heading_rect.width() // 2, r - 5, heading_text)
        
        # Compass tick marks at bottom
        painter.setPen(QPen(self.text_color, 1))
        small_font = QFont("Arial", 7)
        painter.setFont(small_font)
        for hdg_offset in range(-60, 61, 30):
            hdg = (self.heading + hdg_offset) % 360
            angle_rad = hdg_offset * 3.14159 / 180.0
            tick_x = int(angle_rad * r * 0.7)
            tick_y = int(r * 0.85)
            tick_len = 4 if hdg_offset % 30 == 0 else 2
            painter.drawLine(tick_x, tick_y, tick_x, tick_y + tick_len)
            if hdg_offset % 90 == 0:
                label = f"{int(hdg)}"
                painter.drawText(tick_x - 8, tick_y + 18, label)
    
    def sizeHint(self):
        return QSize(200, 200)


class ExecTimeBar(QWidget):
    """Execution time bargraph — current (blue) and peak (red)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.exec_current = 0.0
        self.exec_peak = 0.0
        self.bar_peak_color = QColor(180, 0, 0)
        self.bar_current_color = QColor(0, 120, 255)
        self.setMinimumHeight(24)
        self.setMaximumHeight(28)
        self.setMinimumWidth(200)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def sizeHint(self):
        return QSize(300, 26)

    def set_exec_time(self, current_percent: float, peak_percent: float):
        self.exec_current = max(0.0, min(100.0, current_percent))
        self.exec_peak = max(0.0, min(100.0, peak_percent))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()
        margin = 4
        bar_width = w - 80
        bar_height = h - 6
        bar_x = 30
        bar_y = 3

        painter.setPen(QPen(QColor(80, 80, 80), 1))
        painter.setBrush(QBrush(QColor(20, 20, 20)))
        painter.drawRoundedRect(bar_x, bar_y, bar_width, bar_height, 3, 3)

        peak_w = int(bar_width * self.exec_peak / 100.0)
        if peak_w > 0:
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(self.bar_peak_color))
            painter.drawRoundedRect(bar_x, bar_y, peak_w, bar_height, 3, 3)

        curr_w = int(bar_width * self.exec_current / 100.0)
        if curr_w > 0:
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(self.bar_current_color))
            painter.drawRoundedRect(bar_x, bar_y, curr_w, bar_height, 3, 3)

        font = QFont()
        font.setPointSize(8)
        painter.setFont(font)
        painter.setPen(QPen(Qt.white, 1))
        painter.drawText(bar_x + 4, bar_y + 14, "CPU")
        pct_text = f"{self.exec_current:.0f}% / {self.exec_peak:.0f}%"
        painter.drawText(bar_x + bar_width + 6, bar_y + 14, pct_text)

        # Centered percentage label drawn on top of the bar itself.
        if bar_width > 40:
            painter.setPen(QPen(Qt.white, 1))
            bold = QFont()
            bold.setPointSize(9)
            bold.setBold(True)
            painter.setFont(bold)
            painter.drawText(bar_x, bar_y, bar_width, bar_height,
                            Qt.AlignCenter, f"{self.exec_current:.0f}%")
