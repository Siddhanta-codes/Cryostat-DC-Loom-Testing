import sys
import serial
import serial.tools.list_ports
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel,
    QPushButton, QComboBox, QGridLayout, QHBoxLayout
)
from PyQt5.QtGui import QPixmap, QFont


class WireCheckerGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Cryogenic Wire Loom Checker")
        self.setGeometry(100, 100, 800, 600)

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        # University logo
        image_label = QLabel(self)
        try:
            pixmap = QPixmap("C:/Users/Siddhanta/Desktop/uofg_logo") 
            pixmap = pixmap.scaledToWidth(600)
            image_label.setPixmap(pixmap)
            image_label.setAlignment(Qt.AlignCenter)
            self.layout.addWidget(image_label)
        except:
            pass  

        # COM Port and Baud Layout
        com_layout = QHBoxLayout()

        self.com_label = QLabel("COM Port:")
        self.com_combo = QComboBox()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.com_combo.addItems(ports)
        com_layout.addWidget(self.com_label)
        com_layout.addWidget(self.com_combo)

        self.baud_label = QLabel("Baud Rate:")
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        com_layout.addWidget(self.baud_label)
        com_layout.addWidget(self.baud_combo)

        self.layout.addLayout(com_layout)

        # Create a label for the title
        title_label = QLabel("Cryostat DC Loom Tester", self)
        title_label.setAlignment(Qt.AlignCenter)  # Align the title to the center
        title_label.setFont(QFont("Arial", 16, QFont.Bold))  # Set font and size for the title
        title_label.setContentsMargins(0, 0, 0, 0)  # No extra space around the label
        self.layout.addWidget(title_label)
    
        # Connect Button
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_serial)
        self.layout.addWidget(self.connect_button)

        # Grid for 24 wire status labels
        self.grid = QGridLayout()
        self.labels = []
        for i in range(24):
            lbl = QLabel(f"Wire {i+1}: --")
            lbl.setStyleSheet("color: gray; font-size: 14px;")
            self.labels.append(lbl)
            self.grid.addWidget(lbl, i // 4, i % 4)
        self.layout.addLayout(self.grid)

        # Serial and timer
        self.serial = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial)

        self.expecting_wire = None

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
                            # Update label
                            self.labels[index].setText(f"Wire {self.expecting_wire}: {line}")

                            # Color code based on status
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
