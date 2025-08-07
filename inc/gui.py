import sys
import serial
import serial.tools.list_ports
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel,
    QPushButton, QComboBox, QGridLayout, QHBoxLayout,
    QGroupBox, QSpacerItem, QSizePolicy
)
from PyQt5.QtGui import QPixmap, QFont, QColor


class WireCheckerGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Cryogenic Wire Loom Checker")
        self.setGeometry(100, 100, 900, 750)

        # Modern Style
        self.setStyleSheet("""
            QWidget {
                background-color: qlineargradient(
                    spread:pad, x1:0, y1:0, x2:1, y2:1, 
                    stop:0 #e0eafc, stop:1 #cfdef3
                );
                font-family: Segoe UI;
                font-size: 14px;
            }
            QPushButton {
                background-color: #005a9e;
                color: white;
                padding: 8px 15px;
                font-size: 14px;
                font-weight: bold;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #007acc;
            }
            QComboBox {
                padding: 6px;
                border-radius: 5px;
                background-color: white;
            }
            QLabel {
                margin: 2px;
            }
            QGroupBox {
                border: 2px solid #005a9e;
                border-radius: 10px;
                margin-top: 15px;
                font-weight: bold;
                color: #003366;
                background-color: rgba(255,255,255,0.8);
            }
            QGroupBox:title {
                subcontrol-origin: margin;
                subcontrol-position: top center;
                padding: 0px 5px;
            }
        """)

        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(20, 20, 20, 20)
        self.layout.setSpacing(20)
        self.setLayout(self.layout)

        self.setup_logo()
        self.setup_title()
        self.setup_com_controls()
        self.setup_connect_button()
        self.setup_wire_grid()

        self.serial = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial)

    def setup_logo(self):
        image_label = QLabel(self)
        try:
            pixmap = QPixmap("C:/Users/Siddhanta/Desktop/MSc Project GUI/uofg_logo")
            pixmap = pixmap.scaledToWidth(400, Qt.SmoothTransformation)
            image_label.setPixmap(pixmap)
            image_label.setAlignment(Qt.AlignCenter)
            self.layout.addWidget(image_label)
        except:
            pass

    def setup_title(self):
        title_label = QLabel("Cryostat DC Loom Tester", self)
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setFont(QFont("Segoe UI", 28, QFont.Bold))
        title_label.setStyleSheet("color: #003366; margin-bottom: 20px;")
        self.layout.addWidget(title_label)

    def setup_com_controls(self):
        com_group = QGroupBox("Connection Settings")
        com_layout = QHBoxLayout()
        com_group.setLayout(com_layout)

        self.com_combo = QComboBox()
        self.refresh_ports()
        refresh_btn = QPushButton("⟳")
        refresh_btn.setFixedWidth(40)
        refresh_btn.clicked.connect(self.refresh_ports)

        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("9600")

        com_layout.addWidget(QLabel("COM Port:"))
        com_layout.addWidget(self.com_combo)
        com_layout.addWidget(refresh_btn)
        com_layout.addSpacing(20)
        com_layout.addWidget(QLabel("Baud Rate:"))
        com_layout.addWidget(self.baud_combo)

        self.layout.addWidget(com_group)

    def setup_connect_button(self):
        self.connect_button = QPushButton("Connect")
        self.connect_button.setFixedWidth(150)
        self.connect_button.clicked.connect(self.connect_serial)
        self.layout.addWidget(self.connect_button, alignment=Qt.AlignCenter)

    def setup_wire_grid(self):
        grid_group = QGroupBox("Wire Status Monitor")
        grid_layout = QGridLayout()
        grid_layout.setSpacing(12)
        grid_group.setLayout(grid_layout)

        self.labels = []
        for i in range(24):
            lbl = QLabel(f"● Wire {i+1}: --")
            lbl.setFont(QFont("Segoe UI", 12, QFont.Bold))
            lbl.setFixedWidth(220)
            lbl.setStyleSheet("color: gray; padding: 4px;")
            self.labels.append(lbl)
            grid_layout.addWidget(lbl, i // 4, i % 4)

        self.layout.addWidget(grid_group)

    def refresh_ports(self):
        self.com_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.com_combo.addItems(ports if ports else ["No Ports Found"])

    def connect_serial(self):
        port = self.com_combo.currentText()
        if "No Ports" in port:
            return
        baud = int(self.baud_combo.currentText())
        try:
            self.serial = serial.Serial(port, baud, timeout=1)
            self.timer.start(100)
            self.connect_button.setText("Connected ✓")
            self.connect_button.setEnabled(False)
            self.connect_button.setStyleSheet("background-color: #28a745; color: white;")
        except Exception as e:
            print(f"Connection error: {e}")

    def read_serial(self):
        if self.serial:
            try:
                line = self.serial.readline().decode(errors="ignore").strip()
                if not line.startswith("Wire"):
                    return

                # Example line: "Wire 2 C 35 ohm"
                parts = line.split()
                if len(parts) < 3:
                    return

                try:
                    wire_num = int(parts[1])
                    index = wire_num - 1
                except:
                    return

                status = parts[2].upper()
                if status == "N":
                    self.labels[index].setText(f"🔴 Wire {wire_num}: Not Connected")
                    self.labels[index].setStyleSheet("color: red; font-weight: bold;")

                elif status == "C":
                    if len(parts) >= 5 and parts[4].lower().startswith("ohm"):
                        resistance = parts[3] + " " + parts[4]
                        self.labels[index].setText(f"🟢 Wire {wire_num}: Connected | {resistance}")
                    else:
                        self.labels[index].setText(f"🟢 Wire {wire_num}: Connected")

                    self.labels[index].setStyleSheet("color: green; font-weight: bold;")

            except Exception as e:
                print(f"Error reading/parsing: {e}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = WireCheckerGUI()
    gui.show()
    sys.exit(app.exec_())
