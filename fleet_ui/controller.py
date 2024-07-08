from PySide2 import QtWidgets, QtCore
from PySide2.QtWidgets import QPushButton, QButtonGroup, QLabel, QComboBox, QTableWidgetItem
from PySide2.QtCore import QSize, QRect, Qt, QThread, QObject, Signal, Slot, qDebug
from PySide2.QtGui import QPixmap, QImage
import json, cv2
import numpy as np
from math import floor
from uuid import uuid1

from mainwindow import Ui_MainWindow

import rclpy
from amr_ros2_nodes import PosUpdaterThread, TaskManageThread

CONFIG = json.load(open('resources/conf.json', 'r'))
ORIENTATION = {
	'ORIGINAL': 0,
	'CLOCKWISE90': 1,
	'CLOCKWISE180': 2,
	'CLOCKWISE270': 3
}

class MainWindow_controller(QtWidgets.QMainWindow):
	edit_pos_param = Signal(int, int, float)
	assign_task_signal = Signal(str, str, bool, float, float)

	def __init__(self):
		super().__init__()
		self.ui = Ui_MainWindow()
		self.ui.setupUi(self)
		self.setup()
		self.setWindowState(Qt.WindowMaximized)

	def phyPosToMap(self, phy_x, phy_y):
		map_x = (phy_x - CONFIG['phy_origin_point'][0]) / self.map_resolution
		map_y = (phy_y - CONFIG['phy_origin_point'][1]) / self.map_resolution

		if CONFIG['map_orientation'] == ORIENTATION['CLOCKWISE90']:
			tmp_x = map_x
			map_x = map_y
			map_y = -tmp_x
		elif CONFIG['map_orientation'] == ORIENTATION['CLOCKWISE180']:
			map_x = -map_x
			map_y = -map_y
		elif CONFIG['map_orientation'] == ORIENTATION['CLOCKWISE270']:
			tmp_x = map_x
			map_x = -map_y
			map_y = tmp_x

		map_x = round(self.map_ori_x + map_x)
		map_y = round(self.map_ori_y - map_y)

		return (map_x, map_y)

	def placeButton(self):
		idx = 0
		for button in self.button_group.buttons():
			x, y = self.phyPosToMap(CONFIG['phy_stop_coors'][idx][0], CONFIG['phy_stop_coors'][idx][1])
			button.move(x, y)
			idx += 1

	def setup(self):
		max_w, max_h = QtWidgets.QApplication.desktop().size().toTuple()
		self.ui.tableTask.setMaximumSize(QSize(max_w // 3, max_h))

		self.map_label = QLabel(self.ui.widgetMap)
		self.map_img = QPixmap(CONFIG['map_image'])
		self.map_scale = 1
		self.map_resolution = CONFIG['map_resolution']
		w, h = CONFIG['map_image_shape']
		self.map_label.setPixmap(self.map_img)
		self.ui.widgetConstraint.setMinimumSize(QSize(w, h))
		max_w, max_h = self.ui.widgetConstraint.size().toTuple()
		x = round((max_w - CONFIG['map_image_shape'][0]) / 2)
		y = round((max_h - CONFIG['map_image_shape'][1]) / 2)
		self.ui.widgetMap.setGeometry(x, y, w, h)

		self.map_ori_x, self.map_ori_y = CONFIG['map_origin_point']
		self.phy_ori_x, self.phy_ori_y = CONFIG['phy_origin_point']
		pin_ori_img = cv2.imread(CONFIG['cur_pos_icon'], cv2.IMREAD_UNCHANGED)
		if CONFIG['map_orientation'] == ORIENTATION['CLOCKWISE90']:
			self.pin_img = cv2.rotate(pin_ori_img, cv2.ROTATE_90_CLOCKWISE)
		elif CONFIG['map_orientation'] == ORIENTATION['CLOCKWISE180']:
			self.pin_img = cv2.rotate(pin_ori_img, cv2.ROTATE_180)
		elif CONFIG['map_orientation'] == ORIENTATION['CLOCKWISE270']:
			self.pin_img = cv2.rotate(pin_ori_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
		else:
			self.pin_img = pin_ori_img
		self.pos_icon_offset_x = CONFIG['cur_pos_icon_shape'][0] // 2
		self.pos_icon_offset_y = CONFIG['cur_pos_icon_shape'][1] // 2

		self.button_group = QButtonGroup(self)
		for idx in range(len(CONFIG['display_stops'])):
			x, y, icon = CONFIG['display_stops'][idx]
			button = QPushButton(self.ui.widgetMap)
			button.setText(f'{idx}')
			button.setStyleSheet(f'background-image: url("{icon}");')
			button.setGeometry(QRect(x, y, CONFIG['button_icon_shape'][0], CONFIG['button_icon_shape'][1]))
			self.button_group.addButton(button, idx)

			self.ui.comboBoxGoal.addItem(f'{idx}')

		rclpy.init()

		self.ui.comboBoxBot.addItem('any')
		self.bots = {}
		self.pos_thread = PosUpdaterThread(self.map_ori_x, self.map_ori_y, self.phy_ori_x, self.phy_ori_y, CONFIG['map_resolution'], CONFIG['map_orientation'])
		self.pos_thread.update_signal.connect(self.updatePos)
		self.edit_pos_param.connect(self.pos_thread.editParam)
		self.pos_thread.start()

		self.tasks = {}
		self.task_thread = TaskManageThread()
		self.task_thread.update_signal.connect(self.updateTaskStatus)
		self.assign_task_signal.connect(self.task_thread.assignTask)
		self.ui.pushButtonSendTask.clicked.connect(self.sendTask)
		self.task_thread.start()

	def resizeEvent(self, event):
		max_w, max_h = self.ui.widgetConstraint.size().toTuple()
		scale = min(floor(10 * max_w / CONFIG['map_image_shape'][0]) / 10,
			floor(10 * max_h / CONFIG['map_image_shape'][1]) / 10)

		w = round(CONFIG['map_image_shape'][0] * scale)
		h = round(CONFIG['map_image_shape'][1] * scale)
		x = round((max_w - w) / 2)
		y = round((max_h - h) / 2)
		self.ui.widgetMap.setGeometry(x, y, w, h)

		if scale >= 1 and scale != self.map_scale:
			self.map_ori_x, self.map_ori_y = CONFIG['map_origin_point']
			self.map_ori_x = round(self.map_ori_x * scale)
			self.map_ori_y = round(self.map_ori_y * scale)

			self.map_img = QPixmap(CONFIG['map_image']).scaled(w, h)
			self.map_label.setPixmap(self.map_img)
			self.map_label.resize(w, h)
			self.ui.widgetMap.resize(w, h)
			self.map_resolution = CONFIG['map_resolution'] / scale
			self.map_scale = scale

			self.edit_pos_param.emit(self.map_ori_x, self.map_ori_y, self.map_resolution)
			self.placeButton()

	@Slot(str, int, int, float)
	def updatePos(self, bot_name, map_x, map_y, orientation):
		if self.bots.get(bot_name) is None:
			self.ui.comboBoxBot.addItem(bot_name)
			self.bots[bot_name] = {
				'pin_label': QLabel(self.ui.widgetMap),
				'name_label': QLabel(self.ui.widgetMap)
			}
			pos_label_img = QPixmap.fromImage(QImage(self.pin_img, CONFIG['cur_pos_icon_shape'][0], CONFIG['cur_pos_icon_shape'][1],
				CONFIG['cur_pos_icon_shape'][0]*4, QImage.Format_ARGB32))
			self.bots[bot_name]['pin_label'].setPixmap(pos_label_img)
			self.bots[bot_name]['pin_label'].setMask(pos_label_img.mask())
			self.bots[bot_name]['pin_label'].setGeometry(QRect(self.map_ori_x - self.pos_icon_offset_x, self.map_ori_y - self.pos_icon_offset_y,
				CONFIG['cur_pos_icon_shape'][0], CONFIG['cur_pos_icon_shape'][1]))
			self.bots[bot_name]['pin_label'].show()

			self.bots[bot_name]['name_label'].setText(bot_name)
			self.bots[bot_name]['name_label'].setAlignment(Qt.AlignCenter)
			self.bots[bot_name]['name_label'].setGeometry(QRect(self.map_ori_x - self.pos_icon_offset_x, self.map_ori_y - 13,
				CONFIG['cur_pos_icon_shape'][0], 26))
			self.bots[bot_name]['name_label'].show()
		elif map_x < 0 and self.bots.get(bot_name) is not None:
			self.ui.comboBoxBot.removeItem(self.ui.comboBoxBot.findText(self.bots[bot_name]['name_label'].text()))
			self.bots[bot_name]['pin_label'].setParent(None)
			self.bots[bot_name]['name_label'].setParent(None)
			_ = self.bots.pop(bot_name)
		else:
			rot_mat = cv2.getRotationMatrix2D((self.pos_icon_offset_x, self.pos_icon_offset_y), orientation, 1.0)
			pos_label_img = QPixmap.fromImage(
				QImage(
					cv2.warpAffine(self.pin_img, rot_mat, (CONFIG['cur_pos_icon_shape'][0], CONFIG['cur_pos_icon_shape'][1])),
					CONFIG['cur_pos_icon_shape'][0], CONFIG['cur_pos_icon_shape'][1],
					CONFIG['cur_pos_icon_shape'][0]*4, QImage.Format_ARGB32
				)
			)
			self.bots[bot_name]['pin_label'].clear()
			self.bots[bot_name]['pin_label'].setPixmap(pos_label_img)
			self.bots[bot_name]['pin_label'].setMask(pos_label_img.mask())

			map_x -= self.pos_icon_offset_x
			self.bots[bot_name]['pin_label'].setGeometry(QRect(map_x, map_y - self.pos_icon_offset_y, CONFIG['cur_pos_icon_shape'][0], CONFIG['cur_pos_icon_shape'][1]))
			self.bots[bot_name]['name_label'].setGeometry(QRect(map_x, map_y - 13, CONFIG['cur_pos_icon_shape'][0], 26))
			self.bots[bot_name]['name_label'].raise_()

	@Slot(str, str, str, bool)
	def updateTaskStatus(self, task_uuid, bot_name, status, finished):
		if self.tasks.get(task_uuid) is not None:
			if f'{self.ui.tableTask.item(self.tasks[task_uuid], 1).text()}' == '':
				self.ui.tableTask.item(self.tasks[task_uuid], 1).setText(f'{bot_name}')
			self.ui.tableTask.item(self.tasks[task_uuid], 3).setText(f'{status}')
			if finished:
				if self.ui.tableTask.cellWidget(self.tasks[task_uuid], 0) is not None:
					self.ui.tableTask.removeCellWidget(self.tasks[task_uuid], 0)
				_ = self.tasks.pop(task_uuid)

	def sendTask(self):
		row_idx = self.ui.tableTask.rowCount()
		self.ui.tableTask.setRowCount(row_idx + 1)

		btn = QPushButton(self.ui.tableTask)
		btn.setText('Cancel')
		btn.setStyleSheet('color: #0000FF; text-decoration: underline;')
		btn.clicked.connect(self.cancelTask)

		bot = '' if self.ui.comboBoxBot.currentText() == 'any' else self.ui.comboBoxBot.currentText()
		goal_id = int(self.ui.comboBoxGoal.currentText())
		bot_item = QTableWidgetItem(f'{bot}')
		goal_item = QTableWidgetItem(f'{goal_id}')
		status_item = QTableWidgetItem('')

		self.ui.tableTask.setCellWidget(row_idx, 0, btn)
		self.ui.tableTask.setItem(row_idx, 1, bot_item)
		self.ui.tableTask.setItem(row_idx, 2, goal_item)
		self.ui.tableTask.setItem(row_idx, 3, status_item)

		task_uuid = uuid1().hex
		self.tasks[task_uuid] = row_idx

		self.assign_task_signal.emit(task_uuid, bot, False, CONFIG['phy_stop_coors'][goal_id][0], CONFIG['phy_stop_coors'][goal_id][1])

	def cancelTask(self):
		btn = self.sender()

		row_idx = self.ui.tableTask.indexAt(btn.pos()).row()
		task_uuid = ''
		for k, v in self.tasks.items():
			if row_idx == v:
				task_uuid = k
				break
		self.assign_task_signal.emit(task_uuid, f'{self.ui.tableTask.item(row_idx, 1).text()}', True, 0.0, 0.0)
		btn.setStyleSheet('color: #AAAAAA; text-decoration: underline;')
		self.sender().setEnabled(False)

	def closeEvent(self, event):
		if rclpy.ok():
			rclpy.shutdown()
			self.pos_thread.stop()
			self.pos_thread.wait()
			self.task_thread.stop()
			self.task_thread.wait()

