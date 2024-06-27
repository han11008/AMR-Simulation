import sys
from PySide2 import QtWidgets
from controller import MainWindow_controller

if __name__ == '__main__':
	app = QtWidgets.QApplication(sys.argv)
	window = MainWindow_controller(app.primaryScreen().size().toTuple())
	window.show()
	sys.exit(app.exec_())
