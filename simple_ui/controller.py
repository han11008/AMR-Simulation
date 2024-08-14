from PySide2 import QtWidgets, QtCore
from PySide2.QtWidgets import QPushButton, QButtonGroup, QLabel, QMessageBox
from PySide2.QtCore import QSize, QRect, Qt, QThread, QObject, Signal, Slot, qDebug
from PySide2.QtGui import QPixmap, QImage
import os, subprocess, json, cv2
import numpy as np
from time import time, sleep
from select import select
from math import floor

from mainwindow import Ui_MainWindow

import rclpy
from amr_ros2_nodes import PosUpdaterThread, CamUpdaterThread, PathUpdaterThread, NavDaemonThread

CONFIG = json.load(open('resources/conf.json', 'r'))
ORIENTATION = {
	'ORIGINAL': 0,
	'CLOCKWISE90': 1,
	'CLOCKWISE180': 2,
	'CLOCKWISE270': 3
}

class MainWindow_controller(QtWidgets.QMainWindow):
	edit_pos_param = Signal(int, int, float)
	edit_path_param = Signal(int, int, int, int, float)

	def __init__(self):
		super().__init__()
		self.ui = Ui_MainWindow()
		self.ui.setupUi(self)
		self.confMap()
		self.confButton()
		self.confStat()
		self.confThread()

		self.setWindowState(Qt.WindowMaximized)

	def confMap(self):
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

	def confButton(self):
		self.planToCancel = False # Avoid clicking cancel button multiple times
		self.ui.buttonCancel.clicked.connect(self.cancelTask)
		self.ui.buttonCancel.setEnabled(False)
		self.ui.buttonPatrol.clicked.connect(self.patrol)
		self.ui.buttonHome.clicked.connect(self.goHome)

		self.button_group = QButtonGroup(self)
		for idx in range(len(CONFIG['display_stops'])):
			x, y, icon = CONFIG['display_stops'][idx]
			button = QPushButton(self.ui.widgetMap)
			button.setText(f'{idx}')
			button.setStyleSheet(f'background-image: url("{icon}");')
			button.setGeometry(QRect(x, y, CONFIG['button_icon_shape'][0], CONFIG['button_icon_shape'][1]))
			self.button_group.addButton(button, idx)
		self.button_group.buttonClicked[int].connect(self.navToGoal)

	def confStat(self):
		self.goal_id = 0

		# Create a label with transparent background pointing to where the AMR is
		self.map_pos_label = QLabel(self.ui.widgetMap)
		pin_ori_img = cv2.imread(CONFIG['cur_pos_icon'], cv2.IMREAD_UNCHANGED)
		self.pin_img = pin_ori_img
		if CONFIG['map_orientation'] == ORIENTATION['CLOCKWISE90']:
			self.pin_img = cv2.rotate(pin_ori_img, cv2.ROTATE_90_CLOCKWISE)
		elif CONFIG['map_orientation'] == ORIENTATION['CLOCKWISE180']:
			self.pin_img = cv2.rotate(pin_ori_img, cv2.ROTATE_180)
		elif CONFIG['map_orientation'] == ORIENTATION['CLOCKWISE270']:
			self.pin_img = cv2.rotate(pin_ori_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
		self.pos_icon_offset_x = CONFIG['cur_pos_icon_shape'][0] // 2
		self.pos_icon_offset_y = CONFIG['cur_pos_icon_shape'][1] // 2
		self.map_pos_label.setGeometry(QRect(CONFIG['map_image_shape'][0], CONFIG['map_image_shape'][1], CONFIG['cur_pos_icon_shape'][0], CONFIG['cur_pos_icon_shape'][1]))

		# Create an image layer that marks the plan path
		self.plan_path_label = QLabel(self.ui.widgetMap)
		self.plan_path_label.setGeometry(QRect(0, 0, CONFIG['map_image_shape'][0], CONFIG['map_image_shape'][1]))
		# Order: map image -> plan path -> patrol-stop buttons -> AMR location label
		for button in self.button_group.buttons():
			button.raise_()
		self.map_pos_label.raise_()

	def confThread(self):
		rclpy.init()
		self.get_pos_thread = PosUpdaterThread(self.map_ori_x, self.map_ori_y, self.phy_ori_x, self.phy_ori_y, CONFIG['map_resolution'], CONFIG['map_orientation'])
		self.get_pos_thread.req_signal.connect(self.updatePos)
		self.edit_pos_param.connect(self.get_pos_thread.editParam)
		self.get_pos_thread.start()

		self.cam_frame_path = 'resources/cam.npy'
		self.cam_stride = CONFIG['cam_shape'][0] * CONFIG['cam_shape'][2]
		self.get_cam_thread = CamUpdaterThread(CONFIG['cam_shape'][0], CONFIG['cam_shape'][1], self.cam_frame_path)
		self.get_cam_thread.req_signal.connect(self.updateCam)
		self.get_cam_thread.start()

		self.get_path_thread = PathUpdaterThread(self.map_ori_x, self.map_ori_y, self.phy_ori_x, self.phy_ori_y,
			CONFIG['map_image_shape'][0], CONFIG['map_image_shape'][1], CONFIG['map_resolution'], CONFIG['map_orientation'])
		self.get_path_thread.req_signal.connect(self.updatePath)
		self.edit_path_param.connect(self.get_path_thread.editParam)
		self.get_path_thread.start()

		self.nav_daemon_thread = NavDaemonThread(CONFIG['phy_stop_coors'])
		self.nav_daemon_thread.feedback.connect(self.getNavFeedback)
		self.nav_daemon_thread.update_goal_id.connect(self.updateGoalId)
		self.nav_daemon_thread.start()

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
			self.plan_path_label.resize(w, h)
			self.ui.widgetMap.resize(w, h)
			self.map_resolution = CONFIG['map_resolution'] / scale
			self.map_scale = scale

			self.edit_pos_param.emit(self.map_ori_x, self.map_ori_y, self.map_resolution)
			self.edit_path_param.emit(self.map_ori_x, self.map_ori_y, w, h, self.map_resolution)

			self.placeButton()
			self.plan_path_label.setPixmap(QPixmap())

	# -------------- get_pos_thread --------------
	@Slot(float, float, float)
	def updatePos(self, map_x, map_y, orientation):
		rot_mat = cv2.getRotationMatrix2D((self.pos_icon_offset_x, self.pos_icon_offset_y), orientation, 1.0)
		pos_label_img = QPixmap.fromImage(
			QImage(
				cv2.warpAffine(self.pin_img, rot_mat, (CONFIG['cur_pos_icon_shape'][0], CONFIG['cur_pos_icon_shape'][1])),
				CONFIG['cur_pos_icon_shape'][0], CONFIG['cur_pos_icon_shape'][1],
				CONFIG['cur_pos_icon_shape'][0]*4, QImage.Format_ARGB32
			)
		)
		self.map_pos_label.clear()
		self.map_pos_label.setPixmap(pos_label_img)
		self.map_pos_label.setMask(pos_label_img.mask())
		map_x -= self.pos_icon_offset_x
		map_y -= self.pos_icon_offset_y
		self.map_pos_label.setGeometry(QRect(map_x, map_y, CONFIG['cur_pos_icon_shape'][0], CONFIG['cur_pos_icon_shape'][1]))
	# --------------------------------------------

	# -------------- get_cam_thread --------------
	@Slot(bool)
	def updateCam(self, update):
		if not hasattr(self, 'cam'):
			self.cam = np.memmap(self.cam_frame_path, dtype=np.uint8, mode='r')
		if update:
			frame = QPixmap.fromImage(QImage(self.cam, CONFIG['cam_shape'][0], CONFIG['cam_shape'][1], self.cam_stride, QImage.Format_RGB888))
			self.ui.labelCam.setPixmap(frame.scaled(160, 120))
	# --------------------------------------------

	# -------------- get_path_thread --------------
	@Slot(QPixmap)
	def updatePath(self, plan_path):
		self.plan_path_label.setPixmap(plan_path)
		self.plan_path_label.raise_()
		for button in self.button_group.buttons():
			button.raise_()
		self.map_pos_label.raise_()
	# --------------------------------------------

	# -------------- nav_daemon_thread --------------
	@Slot(str, bool)
	def getNavFeedback(self, msg, en_cancel):
		qDebug(f'[{time()}] status_bar: {msg}')
		self.ui.statusbar.showMessage(msg)
		self.ui.buttonCancel.setEnabled(en_cancel)
		self.planToCancel = False
		if not en_cancel:
			self.plan_path_label.setPixmap(QPixmap())

	@Slot(int)
	def updateGoalId(self, goal_id):
		self.goal_id = goal_id
	# -----------------------------------------------

	def navToGoal(self, button_id, is_patrol=False):
		self.goal_id = button_id
		goal_ids = [button_id]
		if is_patrol:
			num_stops = len(CONFIG['phy_stop_coors'])
			for shift in range(1, num_stops):
				goal_ids.append((button_id + shift) % num_stops)
		self.nav_daemon_thread.assignTask(goal_ids)

	def patrol(self):
		# Start patrolling from the previous goal
		self.navToGoal(self.goal_id, True)

	def goHome(self):
		self.navToGoal(0)

	def cancelTask(self):
		if not self.planToCancel:
			self.planToCancel = True
			self.ui.buttonCancel.setEnabled(False)
			self.nav_daemon_thread.cancelTask()

	def closeEvent(self, event):
		if rclpy.ok():
			rclpy.shutdown()
		self.get_pos_thread.stop()
		self.get_pos_thread.wait()
		self.get_cam_thread.stop()
		self.get_cam_thread.wait()
		self.get_path_thread.stop()
		self.get_path_thread.wait()
		self.nav_daemon_thread.stop()
		self.nav_daemon_thread.wait()
		raise SystemExit

