from PySide2 import QtWidgets, QtCore, QtGui
from PySide2.QtWidgets import QFileDialog, QMessageBox, QLabel, QTableWidgetItem, QPushButton, QWidget
from PySide2.QtCore import QSize, QRect, QMargins, Qt, Signal, Slot
from PySide2.QtGui import QPixmap, QImage, QPainter, QColor
import os, json, re, cv2, yaml
import numpy as np

from mainwindow import Ui_MainWindow

ORIENTATION = {
	'ORIGINAL': 0,
	'CLOCKWISE90': 1,
	'CLOCKWISE180': 2,
	'CLOCKWISE270': 3
}

class MainWindow_controller(QtWidgets.QMainWindow):
	def __init__(self, screen_size):
		super().__init__()
		self.ui = Ui_MainWindow()
		self.ui.setupUi(self)
		max_w, max_h = screen_size
		self.screen_constraint = min(max_w - 400, max_h - 140)
		self.setup()
		self.setWindowFlags(Qt.WindowStaysOnTopHint)
		self.move(0, 0)

	def setup(self):
		self.parent_dir = __file__.split('map_tool')[0]
		self.ros_ws = __file__.split('simple_ui')[0] + 'ws/src/pibot_nav/'

		self.conf = {
			'phy_origin_point': [0.0, 0.0],
			'phy_initial_pose': [0.0, 0.0, 0.0],
			'cam_shape': [160, 120, 3],
			'cur_pos_icon': "resources/pin.png",
			'cur_pos_icon_shape': [51, 51],
			'button_icon_shape': [30, 30]
		}
		self.icon_lt = 'resources/left_top.png'
		self.icon_lb = 'resources/left_bottom.png'
		self.icon_rt = 'resources/right_top.png'
		self.icon_rb = 'resources/right_bottom.png'

		self.ui.actionOpen.triggered.connect(self.openImg)
		self.ui.actionSave.triggered.connect(self.saveConf)
		self.ui.actionAddKeepout.triggered.connect(self.addKeepout)
		self.ui.actionAddKeepout.setEnabled(False)
		self.ui.actionRotateRevert.triggered.connect(lambda: self.mapRotate(ORIENTATION['ORIGINAL']))
		self.ui.actionRotate90.triggered.connect(lambda: self.mapRotate(ORIENTATION['CLOCKWISE90']))
		self.ui.actionRotate180.triggered.connect(lambda: self.mapRotate(ORIENTATION['CLOCKWISE180']))
		self.ui.actionRotate270.triggered.connect(lambda: self.mapRotate(ORIENTATION['CLOCKWISE270']))

		self.ui.widgetMap.setCursor(Qt.CrossCursor)
		self.ui.widgetMap.setMouseTracking(True)
		self.ui.widgetMap.mousePressEvent = self.addStop
		self.stops = []
		self.has_modified = False

		if os.path.exists(f'{self.ros_ws}/map/map.png') and os.path.exists(f'{self.ros_ws}/map/map.yaml'):
			self._openImg(f'{self.ros_ws}/map/map.png')

	def reset(self):
		while self.ui.tablePos.rowCount() > 0:
			self.ui.tablePos.removeRow(0)

		for _ in range(len(self.stops)):
			_, label, _ = self.stops.pop()
			label.hide(); label.clear()
			del label

		if hasattr(self, 'map_ori_x'):
			del self.map_ori_x

		if hasattr(self, 'map_ori_y'):
			del self.map_ori_y

		if hasattr(self, 'map_label'):
			self.map_label.setParent(None)
			del self.map_label

		if hasattr(self, 'inflation_mask_label'):
			self.inflation_mask_label.setParent(None)
			del self.inflation_mask_label

		if hasattr(self, 'childWindow'):
			self.childWindow.close()
			del self.childWindow

		self.has_modified = False

	def mapInflation(self):
		img = cv2.imread(self.img_path)
		h, w, _ = img.shape

		img[img >= 128] = 255
		img[img != 255] = 0
		img = cv2.erode(img, self.dilate_kernel)
		mask = np.full((h, w, 4), 128, dtype=np.uint8)
		mask[:, :, :3] = img
		mask[:, :, -1][img[:, :, 0] == 255] = 0
		mask_pixmap = QPixmap.fromImage(QImage(mask, w, h, w*4, QImage.Format_RGBA8888))
		self.inflation_mask_label.setPixmap(mask_pixmap)
		self.inflation_mask_label.show()		

	def _openImg(self, img_path):
		self.reset()

		self.origin_map_path = img_path
		img = cv2.imread(self.origin_map_path)
		self.origin_h, self.origin_w, _ = img.shape

		img_filename = self.origin_map_path.rsplit('/', 1)[-1]
		keepout_yaml = re.sub(img_filename, 'keepout.png', open(self.origin_map_path.replace('.png', '.yaml'), 'r').read())
		with open(f'{self.ros_ws}/map/keepout.yaml', 'w') as f:
			f.write(keepout_yaml)
		cv2.imwrite(f'{self.ros_ws}/map/keepout.png', np.full_like(img, 255, dtype=np.uint8))

		map_conf = yaml.safe_load(open(f'{self.origin_map_path.rsplit(".", 1)[0]}.yaml', 'r'))

		self.map_scale = min(self.screen_constraint // self.origin_w, self.screen_constraint // self.origin_h, 4)
		self.conf['map_resolution'] = map_conf['resolution'] / self.map_scale
		kernel_size =  1 + 2 * round(0.5 / self.conf['map_resolution']) # try to keep a distance of 0.5m from obstacles
		self.dilate_kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
		w = round(self.origin_w * self.map_scale)
		h = round(self.origin_h * self.map_scale)
		img = cv2.resize(img, (w, h))

		self.conf['map_orientation'] = ORIENTATION['ORIGINAL']
		self.img_path = self.parent_dir + 'resources/map.png'
		cv2.imwrite(self.img_path, img)
		self.conf['map_image'] = 'resources/map.png'
		self.conf['map_image_shape'] = [w, h]

		self.ui.widgetMap.setMinimumSize(QSize(w, h))
		self.ui.widgetMap.setMaximumSize(QSize(w, h))
		self.map_label = QLabel(self.ui.widgetMap)
		self.map_label.setPixmap(QPixmap(self.img_path))
		self.map_label.setGeometry(QRect(0, 0, w, h))
		self.map_label.show()

		self.inflation_mask_label = QLabel(self.ui.widgetMap)
		self.inflation_mask_label.setGeometry(QRect(0, 0, w, h))
		self.inflation_mask_label.setMouseTracking(True)
		self.inflation_mask_label.mouseMoveEvent = self.getMousePos
		self.mapInflation()

		self.occupied = np.zeros((h, w), dtype=bool)
		self.map_ori_x = round(-map_conf['origin'][0] / self.conf['map_resolution'])
		self.map_ori_y = round(h - (-map_conf['origin'][1] / self.conf['map_resolution']))
		self._addStop((self.map_ori_x, self.map_ori_y), (0.0, 0.0))

		self.ui.actionAddKeepout.setEnabled(True)
		self.ui.menuMapRotation.setEnabled(True)

	def openImg(self):
		img_path, _  = QFileDialog.getOpenFileName(self, 'Open file', self.ros_ws + 'map/', 'Image files(*.png)')
		yaml_path = img_path.rsplit('.', 1)[0] + '.yaml'
		if img_path == '' or not os.path.exists(yaml_path):
			QMessageBox.about(self, 'Notification', 'Cannot find map description file(.yaml)')
			return
		self._openImg(img_path)
		QMessageBox.about(self, 'Notification', f'The map is saved at {self.img_path}')

	def saveConf(self):
		tmp_conf = self.conf.copy()
		tmp_conf['phy_stop_coors'] = []
		for idx in range(self.ui.tablePos.rowCount()):
			if idx == 0:
				map_coor = self.ui.tablePos.item(idx, 1).text().split(', ')
				x = int(map_coor[0].split('(')[1])
				y = int(map_coor[1].split(')')[0])
				tmp_conf['map_origin_point'] = [x, y]
			phy_coor = self.ui.tablePos.item(idx, 2).text().split(', ')
			x = float(phy_coor[0].split('(')[1])
			y = float(phy_coor[1].split(')')[0])
			tmp_conf['phy_stop_coors'].append([x, y])

		tmp_conf['display_stops'] = []
		for idx in range(len(self.stops)):
			_, _, info = self.stops[idx]
			tmp_conf['display_stops'].append(info)

		conf_path = open(self.parent_dir + 'resources/conf.json', 'w')
		json.dump(tmp_conf, conf_path, indent=4)
		conf_path.close()
		self.has_modified = False
		QMessageBox.about(self, 'Notification', f'The config file is saved at {conf_path.name}')

	def addKeepout(self):
		self.childWindow = DrawMap(self.img_path, self.dilate_kernel)
		self.childWindow.update_keepout_map.connect(self.saveKeepoutMap)
		self.childWindow.show()
		self.childWindow.raise_()

	def mapRotate(self, rotation_type):
		img = cv2.imread(self.origin_map_path)
		cv2.imwrite(f'{self.ros_ws}/map/keepout.png', np.full_like(img, 255, dtype=np.uint8))
		w = round(self.origin_w * self.map_scale)
		h = round(self.origin_h * self.map_scale)
		img = cv2.resize(img, (w, h))
		self.conf['map_orientation'] = rotation_type
		map_conf = yaml.safe_load(open(f'{self.origin_map_path.rsplit(".", 1)[0]}.yaml', 'r'))
		self.map_ori_x = round(-map_conf['origin'][0] / self.conf['map_resolution'])
		self.map_ori_y = round(h - (-map_conf['origin'][1] / self.conf['map_resolution']))
		cx = w // 2
		cy = h // 2
		if rotation_type == ORIENTATION['CLOCKWISE90']:
			img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
			tmp_x = self.map_ori_x
			self.map_ori_x = -(self.map_ori_y - cy) + cy
			self.map_ori_y = (tmp_x - cx) + cx
		elif rotation_type == ORIENTATION['CLOCKWISE180']:
			img = cv2.rotate(img, cv2.ROTATE_180)
			self.map_ori_x = -(self.map_ori_x - cx) + cx
			self.map_ori_y = -(self.map_ori_y - cy) + cy
		elif rotation_type == ORIENTATION['CLOCKWISE270']:
			img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
			tmp_x = self.map_ori_x
			self.map_ori_x = (self.map_ori_y - cy) + cy
			self.map_ori_y = -(tmp_x - cx) + cx
		cv2.imwrite(self.img_path, img)
		h, w, _ = img.shape
		self.conf['map_image_shape'] = [w, h]

		self.ui.widgetMap.setMinimumSize(QSize(w, h))
		self.ui.widgetMap.setMaximumSize(QSize(w, h))
		self.map_label.setPixmap(QPixmap(self.img_path))
		self.map_label.setGeometry(QRect(0, 0, w, h))
		self.inflation_mask_label.setGeometry(QRect(0, 0, w, h))
		self.mapInflation()

		while self.ui.tablePos.rowCount() > 0:
			self.ui.tablePos.removeRow(0)

		for _ in range(len(self.stops)):
			_, label, _ = self.stops.pop()
			label.hide(); label.clear()
			del label

		self.occupied = np.zeros((h, w), dtype=bool)
		self._addStop((self.map_ori_x, self.map_ori_y), (0.0, 0.0))

	def mapToPhy(self, map_coor):
		map_x, map_y = map_coor
		if hasattr(self, 'map_ori_x'):
			phy_x = round((map_x - self.map_ori_x) * self.conf['map_resolution'], 4)
			phy_y = round((self.map_ori_y - map_y) * self.conf['map_resolution'], 4)

			if self.conf['map_orientation'] == ORIENTATION['CLOCKWISE90']:
				tmp_x = phy_x
				phy_x = -phy_y
				phy_y = tmp_x
			elif self.conf['map_orientation'] == ORIENTATION['CLOCKWISE180']:
				phy_x = -phy_x
				phy_y = -phy_y
			elif self.conf['map_orientation'] == ORIENTATION['CLOCKWISE270']:
				tmp_x = phy_x
				phy_x = phy_y
				phy_y = -tmp_x

			return (phy_x, phy_y)
		else:
			self.map_ori_x = map_x
			self.map_ori_y = map_y
			return (0.0, 0.0)

	def getMousePos(self, event):
		map_coor = event.pos().toTuple()
		log = f'Map: {map_coor}'
		if hasattr(self, 'map_ori_x'):
			phy_coor = self.mapToPhy(map_coor)
			log += f' => Physical: {phy_coor}'
		self.ui.statusbar.showMessage(log)

	def _addStop(self, map_coor, phy_coor):
		map_coor_item = QTableWidgetItem(f'{map_coor}')
		phy_coor_item = QTableWidgetItem(f'{phy_coor}')

		# Determine how the stop is going to shown on the map
		icon_x, icon_y = map_coor
		left_bound = icon_x - self.conf['button_icon_shape'][0]
		right_bound = icon_x + self.conf['button_icon_shape'][0]
		top_bound = icon_y - self.conf['button_icon_shape'][1]
		bottom_bound = icon_y + self.conf['button_icon_shape'][1]
		avaiables = []
		if left_bound >= 0:
			if top_bound >= 0:
				avaiables.append('rb')
			if bottom_bound < self.conf['map_image_shape'][1]:
				avaiables.append('rt')			
		if right_bound < self.conf['map_image_shape'][0]:
			if top_bound >= 0:
				avaiables.append('lb')
			if bottom_bound < self.conf['map_image_shape'][1]:
				avaiables.append('lt')
		icon = None
		while avaiables:
			cand = avaiables.pop()
			if cand == 'lt':
				if np.count_nonzero(self.occupied[icon_y:bottom_bound+1, icon_x:right_bound+1]) == 0:
					self.occupied[icon_y:bottom_bound+1, icon_x:right_bound+1] = True
					icon = self.icon_lt
					break
			elif cand == 'lb':
				if np.count_nonzero(self.occupied[top_bound:icon_y+1, icon_x:right_bound+1]) == 0:
					self.occupied[top_bound:icon_y+1, icon_x:right_bound+1] = True
					icon_y = top_bound
					icon = self.icon_lb
					break
			elif cand == 'rt':
				if np.count_nonzero(self.occupied[icon_y:bottom_bound+1, left_bound:icon_x+1]) == 0:
					self.occupied[icon_y:bottom_bound+1, left_bound:icon_x+1] = True
					icon_x = left_bound
					icon = self.icon_rt
					break
			else:
				if np.count_nonzero(self.occupied[top_bound:icon_y+1, left_bound:icon_x+1]) == 0:
					self.occupied[top_bound:icon_y+1, left_bound:icon_x+1] = True
					icon_x = left_bound
					icon_y = top_bound
					icon = self.icon_rb
					break

		if icon is None:
			QMessageBox.warning(self, ' ', 'The patrol stop is not created because it is too close to other stops.')
			return

		# Add the stop to list
		row_idx = self.ui.tablePos.rowCount()
		self.ui.tablePos.setRowCount(row_idx + 1)
		btn = QPushButton(self.ui.tablePos)
		btn.setText('DEL')
		if row_idx == 0:
			btn.setStyleSheet('color: #AAAAAA; text-decoration: underline;')
			btn.setEnabled(False)
		else:
			btn.setStyleSheet('color: #0000FF; text-decoration: underline;')
		btn.clicked.connect(self.delStop)
		self.ui.tablePos.setCellWidget(row_idx, 0, btn)
		self.ui.tablePos.setItem(row_idx, 1, map_coor_item)
		self.ui.tablePos.setItem(row_idx, 2, phy_coor_item)

		# Show the stop on the map
		new_stop = QLabel(self.ui.widgetMap)
		new_stop.setAlignment(Qt.AlignCenter)
		new_stop.setText(f'{row_idx}')
		new_stop.setStyleSheet(f'background-image: url("{self.parent_dir + icon}");')
		new_stop.setGeometry(QRect(icon_x, icon_y, self.conf['button_icon_shape'][0], self.conf['button_icon_shape'][1]))
		new_stop.show()
		self.stops.append((f'{map_coor}', new_stop, [icon_x, icon_y, icon]))

	def addStop(self, event):
		map_coor = event.pos().toTuple()
		phy_coor = self.mapToPhy(map_coor)
		self._addStop(map_coor, phy_coor)
		self.has_modified = True
		self.ui.menuMapRotation.setEnabled(False)	

	def delStop(self):
		btn = self.sender()

		# Remove the stop from list
		row_idx = self.ui.tablePos.indexAt(btn.pos()).row()
		map_coor = self.ui.tablePos.item(row_idx, 1).text()
		self.ui.tablePos.removeRow(row_idx)

		# Remove the stop from the map
		rm_idx = None
		for idx in range(len(self.stops)):
			if self.stops[idx][0] == map_coor:
				rm_idx = idx
			if rm_idx != None:
				new_idx = int(self.stops[idx][1].text()) - 1
				self.stops[idx][1].setText(f'{new_idx}')
		_, label, stop_info = self.stops.pop(rm_idx)
		label.setParent(None)
		# (x, y) is the left-top point of a icon
		x, y, icon = stop_info
		self.occupied[y : (y + self.conf['button_icon_shape'][1] + 1), x : (x + self.conf['button_icon_shape'][0] + 1)] = False
		self.has_modified = True

		if len(self.stops) == 1:
			self.ui.menuMapRotation.setEnabled(True)

	@Slot(QPixmap)
	def saveKeepoutMap(self, keepout_img):
		if keepout_img.size().toTuple() != (0, 0):
			update_keepout = keepout_img.toImage()
			update_keepout.convertTo(QImage.Format_RGB888)
			update_keepout = np.ndarray((keepout_img.height(), keepout_img.width(), 3), buffer=update_keepout.constBits(), strides=[update_keepout.bytesPerLine(), 3, 1], dtype=np.uint8).copy()

			map_img = cv2.imread(self.img_path)
			map_img[update_keepout == 0] = 0
			self.img_path = self.parent_dir + 'resources/keepout.png'
			cv2.imwrite(self.img_path, map_img)

			if self.conf['map_orientation'] == ORIENTATION['CLOCKWISE90']:
				update_keepout = cv2.rotate(update_keepout, cv2.ROTATE_90_COUNTERCLOCKWISE)
			elif self.conf['map_orientation'] == ORIENTATION['CLOCKWISE180']:
				update_keepout = cv2.rotate(update_keepout, cv2.ROTATE_180)
			elif self.conf['map_orientation'] == ORIENTATION['CLOCKWISE270']:
				update_keepout = cv2.rotate(update_keepout, cv2.ROTATE_90_CLOCKWISE)
			update_keepout = cv2.resize(update_keepout, (self.origin_w, self.origin_h))

			keepout_map = cv2.imread(f'{self.ros_ws}/map/keepout.png')
			keepout_map[update_keepout == 0] = 0
			cv2.imwrite(f'{self.ros_ws}/map/keepout.png', keepout_map)

			self.map_label.setPixmap(QPixmap(self.img_path))
			self.map_label.show()

			self.mapInflation()

		del self.childWindow

	def closeEvent(self, event):
		if hasattr(self, 'childWindow'):
			self.childWindow.close()
		if self.has_modified and hasattr(self, 'map_ori_x') and (QMessageBox.StandardButton.Yes == QMessageBox.question(self, ' ', 'Save configuration?')):
			self.saveConf()

class DrawMap(QWidget):
	update_keepout_map = Signal(QPixmap)

	def __init__(self, img_path, dilate_kernel):
		super().__init__()
		self.setWindowTitle('Add keepout zones')
		self.setWindowFlags(Qt.WindowStaysOnTopHint)
		self.move(0, 0)
		self.dilate_kernel = dilate_kernel
		self.inflation = (len(dilate_kernel) - 1) // 2

		# map image to draw and show in the widget
		self.pix_img = QPixmap(img_path)
		self.w, self.h = self.pix_img.size().toTuple()
		self.setMinimumSize(QSize(self.w, self.h))
		self.setMaximumSize(QSize(self.w, self.h))

		self.keepout_img = QPixmap(self.w, self.h)
		self.keepout_img.fill(Qt.white)

		self.mapInflation()

		self.rect_start = self.rect_end = None
		self.finish_draw = False
		self.has_modified = False
		self.setCursor(Qt.CrossCursor)
		self.setMouseTracking(True)

	def mapInflation(self):
		binary_img = self.pix_img.toImage()
		binary_img.convertTo(QImage.Format_RGB888)
		binary_img = np.ndarray((self.h, self.w, 3), buffer=binary_img.constBits(), strides=[binary_img.bytesPerLine(), 3, 1], dtype=np.uint8).copy()
		binary_img[binary_img >= 128] = 255
		binary_img[binary_img != 255] = 0
		self.pix_img = QPixmap.fromImage(QImage(binary_img, self.w, self.h, self.w*3, QImage.Format_RGB888))

		inflated = cv2.erode(binary_img, self.dilate_kernel)
		mask_img = np.full((self.h, self.w, 4), 128, dtype=np.uint8)
		mask_img[:, :, :3] = inflated
		mask_img[:, :, -1][inflated[:, :, 0] == 255] = 0
		self.mask = QPixmap.fromImage(QImage(mask_img, self.w, self.h, self.w*4, QImage.Format_RGBA8888))
		self.update()

	def mousePressEvent(self, event):
		if event.button() == Qt.MouseButton.LeftButton:
			self.rect_start = self.rect_end = event.pos()
			self.update()
		elif event.button() == Qt.MouseButton.RightButton:
			self.rect_start = self.rect_end = None
			self.update()

	def mouseMoveEvent(self, event):
		if self.rect_start is not None:
			self.rect_end = event.pos()
			self.update()

	def mouseReleaseEvent(self, event):
		if event.button() == Qt.MouseButton.LeftButton:
			if self.rect_start is not None:
				self.finish_draw = True
				self.rect_end = event.pos()
				self.update()

	def paintEvent(self, event):
		painter_screen = QPainter(self)
		painter_screen.drawPixmap(QtCore.QRect(0, 0, self.w, self.h), self.pix_img)
		painter_screen.drawPixmap(QtCore.QRect(0, 0, self.w, self.h), self.mask)
		if self.rect_start is not None:
			if self.rect_start.x() < self.rect_end.x():
				left = self.rect_start.x()
				right = self.rect_end.x()
			else:
				left = self.rect_end.x()
				right = self.rect_start.x()
			if self.rect_start.y() < self.rect_end.y():
				top = self.rect_start.y()
				bottom = self.rect_end.y()
			else:
				top = self.rect_end.y()
				bottom = self.rect_start.y()
			keepout_zone = QRect(left, top, right-left, bottom-top)
			painter_screen.setBrush(QColor(0, 0, 0, 255))
			painter_screen.drawRect(keepout_zone)

			inflated = keepout_zone.marginsAdded(QMargins(self.inflation, self.inflation, self.inflation, self.inflation))
			painter_screen.setBrush(QColor(255, 0, 0, 80))
			painter_screen.setPen(QColor(0, 0, 0, 0))
			painter_screen.drawRect(inflated)
		painter_screen.end()
		if self.finish_draw:
			painter_img = QPainter(self.pix_img)
			painter_img.setBrush(QColor(0, 0, 0, 255))
			painter_img.drawRect(QRect(self.rect_start, self.rect_end))
			painter_img.end()

			painter_keepout_img = QPainter(self.keepout_img)
			painter_keepout_img.setBrush(QColor(0, 0, 0, 255))
			painter_keepout_img.drawRect(QRect(self.rect_start, self.rect_end))
			painter_keepout_img.end()

			self.mapInflation()
			self.finish_draw = False
			self.rect_start = self.rect_end = None
			self.has_modified = True

	def closeEvent(self, event):
		if self.has_modified and (QMessageBox.StandardButton.Yes == QMessageBox.question(self, ' ', 'Save keepout map?')):
			self.update_keepout_map.emit(self.keepout_img)
		else:
			self.update_keepout_map.emit(QPixmap())

