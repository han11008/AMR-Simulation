import sys
from PySide2 import QtWidgets
from controller import MainWindow_controller

if __name__ == '__main__':
	app = QtWidgets.QApplication(sys.argv)
	window = MainWindow_controller()
	window.show()
	sys.exit(app.exec_())
