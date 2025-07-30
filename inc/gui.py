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
        self.setGeometry(100, 100, 850, 700)

        self.setStyleSheet("""
            QWidget {
                background-color: #f0f0f0;
                font-family: Arial;
                font-size: 14px;
            }
            QPushButton {
                background-color: #005a9e;
                color: white;
                padding: 8px;
                font-size: 14px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #007acc;
            }
            QComboBox {
                padding: 4px;
            }
            QLabel {
                margin: 2px;
            }
        """)

        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(20, 20, 20, 20)
        self.layout.setSpacing(15)
        self.setLayout(self.layout)

        self.setup_logo()
        self.setup_title()
        self.setup_com_controls()
        self.setup_connect_button()
        self.setup_wire_grid()

        self.serial = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial)
        self.expecting_wire = None

    def setup_logo(self):
        image_label = QLabel(self)
        try:
            pixmap = QPixmap("C:/Users/Siddhanta/Desktop/uofg_logo")
            pixmap = pixmap.scaledToWidth(500, Qt.SmoothTransformation)
            image_label.setPixmap(pixmap)
            image_label.setAlignment(Qt.AlignCenter)
            self.layout.addWidget(image_label)
        except:
            pass  # Skip logo if not found

    def setup_title(self):
        title_label = QLabel("Cryostat DC Loom Tester", self)
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setFont(QFont("Times New Roman", 24, QFont.Bold))
        title_label.setStyleSheet("color: #003366; margin-bottom: 10px;")
        self.layout.addWidget(title_label)

    def setup_com_controls(self):
        com_group = QGroupBox("Connection Settings")
        com_layout = QHBoxLayout()
        com_group.setLayout(com_layout)

        self.com_combo = QComboBox()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.com_combo.addItems(ports)

        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("9600")

        com_layout.addWidget(QLabel("COM Port:"))
        com_layout.addWidget(self.com_combo)
        com_layout.addSpacing(20)
        com_layout.addWidget(QLabel("Baud Rate:"))
        com_layout.addWidget(self.baud_combo)

        self.layout.addWidget(com_group)

    def setup_connect_button(self):
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_serial)
        self.layout.addWidget(self.connect_button, alignment=Qt.AlignCenter)

    def setup_wire_grid(self):
        grid_group = QGroupBox("Wire Status Monitor")
        grid_layout = QGridLayout()
        grid_layout.setSpacing(10)
        grid_group.setLayout(grid_layout)

        self.labels = []
        for i in range(24):
            lbl = QLabel(f"Wire {i+1}: --")
            lbl.setFixedWidth(200)
            lbl.setStyleSheet("color: gray; font-size: 13px; padding: 4px;")
            self.labels.append(lbl)
            grid_layout.addWidget(lbl, i // 4, i % 4)

        self.layout.addWidget(grid_group)

    def connect_serial(self):
        port = self.com_combo.currentText()
        baud = int(self.baud_combo.currentText())
        try:
            self.serial = serial.Serial(port, baud, timeout=1)
            self.timer.start(100)
            self.connect_button.setText("Connected")
            self.connect_button.setEnabled(False)
        except Exception as e:
            print(f"Connection error: {e}")

    def read_serial(self):
        if self.serial and self.serial.in_waiting:
            try:
                data = self.serial.read(self.serial.in_waiting).decode(errors="ignore")
                lines = data.splitlines()

                for line in lines:
                    line = line.strip()

                    if line.startswith("Wire") and "ADC Value" in line:
                        try:
                            parts = line.split()
                            wire_num = int(parts[1])
                            self.expecting_wire = wire_num
                        except:
                            self.expecting_wire = None
                            continue

                    elif self.expecting_wire:
                        index = self.expecting_wire - 1
                        if 0 <= index < 24:
                            self.labels[index].setText(f"Wire {self.expecting_wire}: {line}")
                            # Color based on status
                            if "Connected" in line:
                                self.labels[index].setStyleSheet("color: green; font-weight: bold;")
                            elif "Not Connected" in line:
                                self.labels[index].setStyleSheet("color: red; font-weight: bold;")
                            else:
                                self.labels[index].setStyleSheet("color: gray;")

                        self.expecting_wire = None

            except Exception as e:
                print(f"Error reading/parsing: {e}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = WireCheckerGUI()
    gui.show()
    sys.exit(app.exec_())
