from PyQt5.QtWidgets import QComboBox, QMainWindow, QApplication, QWidget, QVBoxLayout
from PyQt5.QtGui import QIcon
import sys


class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()

        combobox1 = QComboBox()
        combobox1.addItem('One')
        combobox1.addItem('Two')
        combobox1.addItem('Three')
        combobox1.addItem('Four')

        combobox2 = QComboBox()
        combobox2.addItems(['One', 'Two', 'Three', 'Four'])

        combobox3 = QComboBox()
        combobox3.addItems(['One', 'Two', 'Three', 'Four'])
        combobox3.insertItem(2, 'Hello!')

        combobox4 = QComboBox()
        combobox4.addItems(['One', 'Two', 'Three', 'Four'])
        combobox4.insertItems(2, ['Hello!', 'again'])

        combobox5 = QComboBox()
        icon_penguin = QIcon('animal-penguin.png')
        icon_monkey = QIcon('animal-monkey.png')
        icon_bauble = QIcon('bauble.png')
        combobox5.addItem(icon_penguin, 'Linux')
        combobox5.addItem(icon_monkey, 'Monkeyix')
        combobox5.insertItem(1, icon_bauble, 'Baublix')

        layout = QVBoxLayout()
        layout.addWidget(combobox1)
        layout.addWidget(combobox2)
        layout.addWidget(combobox3)
        layout.addWidget(combobox4)
        layout.addWidget(combobox5)

        container = QWidget()
        container.setLayout(layout)

        self.setCentralWidget(container)

    def current_text_changed(self, s):
        print("Current text: ", s)


app = QApplication(sys.argv)
w = MainWindow()
w.show()
app.exec_()