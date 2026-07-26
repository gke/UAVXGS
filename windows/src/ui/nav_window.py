# ui/nav_window.py
"""
UAVX Navigation Window
Mission planning with map and telemetry integration
"""

import sys
import os
import json
import struct
import urllib.parse
from datetime import datetime

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from protocol_enums import PacketTag
from protocol_constants import NAV_STATE_NAMES
from packet_parser import OriginData
from models.waypoint import Waypoint, WP_ACTION_VALUES, WP_ACTIONS

try:
    import folium
    from folium import plugins
    FOLIUM_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: Folium not available: {e}")
    FOLIUM_AVAILABLE = False

try:
    from PyQt5.QtWebEngineWidgets import QWebEngineView
    WEBENGINE_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: PyQtWebEngine not available: {e}")
    WEBENGINE_AVAILABLE = False
    QWebEngineView = None


class NavWindow(QMainWindow):
    """Navigation/Mission Planning Window"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.waypoints: list[Waypoint] = []
        self.aircraft_pos = None
        self.home_pos = None
        self.map_zoom = 13
        self.auto_follow = True
        self._read_queue = []
        self._read_pending = False
        self._origin_retries = 0

        self._nav_state_label = None
        self._nav_batt_volts = None
        self._nav_batt_current = None
        self._nav_batt_mah = None
        self._nav_curr_wp = None
        self._nav_wp_dist = None
        self._nav_wp_brg = None
        self._nav_wp_xte = None
        self._nav_gps_alt = None
        self._nav_gps_lat = None
        self._nav_gps_lon = None
        self._nav_gps_vel = None
        self._nav_gps_sats = None
        self._nav_alt_press = None
        self._nav_alt_kf = None
        self._nav_alt_roc = None
        self._nav_alt_desired = None
        self._nav_mission_time = None
        self._nav_dist_traveled = None
        self._distance_traveled = 0.0
        self._last_pos_for_dist = None
        self._override_center = None
        self._override_zoom = None
        self._pending_wp_action = None
        self._active_wp_action = None

        self.setWindowTitle("UAVX Navigation - Mission Planning")
        self.setMinimumSize(1200, 768)

        print(f"Folium available: {FOLIUM_AVAILABLE}")

        self.setup_ui()
        self.setup_connections()
        self.load_default_mission()

        self._position_ready = False
        self._startup_origin_retries = 0
        self._show_waiting_for_gps()
        QTimer.singleShot(100, self._start_origin_acquisition)

        self.telemetry_timer = QTimer()
        self.telemetry_timer.timeout.connect(self.update_from_telemetry)
        self.telemetry_timer.start(500)

        self._map_update_timer = QTimer()
        self._map_update_timer.timeout.connect(self._update_aircraft_on_map)
        self._map_update_timer.start(3000)

        self._map_rebuild_key = None
        self._last_pos_update = 0

    def _show_waiting_for_gps(self):
        self.map_widget.setHtml("""<html><body style="font-family: Arial; text-align: center; padding: 80px 20px;">
            <h2 style="color: #555;">Waiting for GPS Position</h2>
            <p style="color: #888; font-size: 14px;">
                The map will appear once a valid position is received from the Flight Controller.
            </p>
            <p style="color: #aaa; font-size: 12px;">
                If this persists, check that the FC is powered and has a GPS fix
                (or emulation is enabled in Config1).
            </p>
        </body></html>""")
        self.status_label.setText("Waiting for GPS position from FC...")

    def _show_gps_timeout(self):
        self.map_widget.setHtml("""<html><body style="font-family: Arial; text-align: center; padding: 80px 20px;">
            <h2 style="color: #c0392b;">GPS Position Timeout</h2>
            <p style="color: #888; font-size: 14px;">
                No position data received from the Flight Controller.
            </p>
            <p style="color: #aaa; font-size: 12px;">
                Check the connection and ensure emulation or GPS is active.<br>
                Use <b>Set Home</b>, <b>From GPS</b>, or <b>Search</b> to set a position manually.
            </p>
        </body></html>""")
        self.status_label.setText("Timeout waiting for GPS position")

    def _start_origin_acquisition(self):
        parent = self.parent()
        if parent and hasattr(parent, 'send_request'):
            self._startup_origin_retries = 0
            parent.send_request(PacketTag.ORIGIN, 0xFF, 0)
            QTimer.singleShot(200, self._poll_startup_origin)
        else:
            self._startup_origin_retries = 0
            QTimer.singleShot(200, self._poll_startup_origin)

    def _poll_startup_origin(self):
        if self._position_ready:
            return
        parent = self.parent()

        if parent:
            flight_data = getattr(parent, 'flight_data', None)
            if flight_data:
                lat = getattr(flight_data, 'gps_lat', 0)
                lon = getattr(flight_data, 'gps_lon', 0)
                if lat != 0 and lon != 0:
                    self._on_position_ready(lat, lon)
                    return

        self._startup_origin_retries += 1
        if self._startup_origin_retries > 25:
            self._show_gps_timeout()
            return

        QTimer.singleShot(200, self._poll_startup_origin)

    def _on_position_ready(self, lat, lon):
        self._position_ready = True
        self.home_pos = (lat, lon)
        self.home_lat.setText(f"{lat:.6f}")
        self.home_lon.setText(f"{lon:.6f}")
        self.aircraft_pos = (lat, lon)

        self.waypoints = []
        self.update_wp_table()
        self.update_map()
        self.status_label.setText(f"Position acquired: {lat:.6f}, {lon:.6f}")

    def _search_location(self):
        query = self.search_edit.text().strip()
        if not query:
            return
        self.status_label.setText(f"Searching for '{query}'...")
        try:
            import requests
            resp = requests.get(
                "https://nominatim.openstreetmap.org/search",
                params={"q": query, "format": "json", "limit": 1}
            )
            if resp.status_code == 200:
                data = resp.json()
                if data:
                    lat = float(data[0]["lat"])
                    lon = float(data[0]["lon"])
                    self.home_pos = (lat, lon)
                    self.home_lat.setText(f"{lat:.6f}")
                    self.home_lon.setText(f"{lon:.6f}")
                    self.map_zoom = 14
                    self.update_map()
                    self.status_label.setText(f"Found: {data[0].get('display_name', query)}")
                else:
                    self.status_label.setText(f"No results for '{query}'")
            else:
                self.status_label.setText("Search failed — check connection")
        except ImportError:
            self.status_label.setText("Search requires 'requests' library (pip install requests)")
        except Exception as e:
            self.status_label.setText(f"Search error: {e}")

    def _on_map_title_changed(self, title):
        if title.startswith('WP:'):
            if self._active_wp_action:
                action = self._active_wp_action
                self._clear_wp_selection()
                if WEBENGINE_AVAILABLE and FOLIUM_AVAILABLE:
                    self.map_widget.page().runJavaScript(
                        "getMapCenter()",
                        lambda r: self._place_wp_from_center(r, action)
                    )
            else:
                try:
                    parts = title[3:].split(',')
                    lat = float(parts[0])
                    lon = float(parts[1])
                    self.add_waypoint("Via", lat, lon)
                except (ValueError, IndexError):
                    pass
        elif title.startswith('WPDRAG:'):
            try:
                parts = title[7:].split(',')
                idx, lat, lon = int(parts[0]), float(parts[1]), float(parts[2])
                row = idx - 1
                if 0 <= row < len(self.waypoints):
                    self.waypoints[row].lat = lat
                    self.waypoints[row].lon = lon
                    self.update_wp_table()
                    self.status_label.setText(f"WP {idx} moved to {lat:.6f}, {lon:.6f}")
                    QTimer.singleShot(0, self._preserve_and_rebuild)
            except (ValueError, IndexError):
                pass

    def _on_map_loaded(self, ok):
        if not ok:
            return
        self._override_center = None
        self._override_zoom = None
        if self.aircraft_pos:
            p = self.aircraft_pos
            self.map_widget.page().runJavaScript(f"initAircraftMarker({p[0]}, {p[1]})")
        if self.home_pos:
            p = self.home_pos
            self.map_widget.page().runJavaScript(f"initHomeMarker({p[0]}, {p[1]})")
        self.map_widget.page().runJavaScript("if (window._wpData) initWaypointMarkers(window._wpData)")

    def setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        toolbar = QHBoxLayout()
        self.load_btn = QPushButton("Load")
        toolbar.addWidget(self.load_btn)
        self.save_btn = QPushButton("Save")
        toolbar.addWidget(self.save_btn)
        self.clear_btn = QPushButton("Clear")
        toolbar.addWidget(self.clear_btn)

        sep = QFrame()
        sep.setFrameShape(QFrame.VLine)
        toolbar.addWidget(sep)

        self.read_btn = QPushButton("Read from UAVX")
        toolbar.addWidget(self.read_btn)
        self.write_btn = QPushButton("Write to UAVX")
        toolbar.addWidget(self.write_btn)

        sep2 = QFrame()
        sep2.setFrameShape(QFrame.VLine)
        toolbar.addWidget(sep2)

        self.follow_check = QCheckBox("Follow Aircraft")
        self.follow_check.setChecked(True)
        toolbar.addWidget(self.follow_check)

        self.zoom_in_btn = QPushButton("+")
        self.zoom_in_btn.setFixedWidth(30)
        toolbar.addWidget(self.zoom_in_btn)

        self.zoom_out_btn = QPushButton("-")
        self.zoom_out_btn.setFixedWidth(30)
        toolbar.addWidget(self.zoom_out_btn)

        toolbar.addStretch()
        self.search_edit = QLineEdit()
        self.search_edit.setPlaceholderText("Search location...")
        self.search_edit.setMaximumWidth(180)
        self.search_edit.returnPressed.connect(self._search_location)
        toolbar.addWidget(self.search_edit)
        self.status_label = QLabel("Ready")
        toolbar.addWidget(self.status_label)
        main_layout.addLayout(toolbar)

        v_splitter = QSplitter(Qt.Vertical)
        main_layout.addWidget(v_splitter)

        # --- Top: map ---
        if WEBENGINE_AVAILABLE:
            self.map_widget = QWebEngineView()
            self.map_widget.titleChanged.connect(self._on_map_title_changed)
        else:
            self.map_widget = QLabel("Map not available - install PyQtWebEngine")
            self.map_widget.setAlignment(Qt.AlignCenter)
            self.map_widget.setStyleSheet("font-size: 14px; color: #666; padding: 50px; background-color: #f8f8f8;")
        self.map_widget.setMinimumHeight(400)
        v_splitter.addWidget(self.map_widget)

        # --- Bottom: info boxes (left) + WP panel (right) ---
        bot_panel = QWidget()
        bot_layout = QVBoxLayout(bot_panel)
        bot_layout.setContentsMargins(0, 0, 0, 0)

        h_splitter = QSplitter(Qt.Horizontal)

        # --- Left panel: telemetry info boxes ---
        left_panel = QWidget()
        left_panel.setMinimumWidth(220)
        left_layout = QVBoxLayout(left_panel)
        left_layout.setSpacing(4)
        left_layout.setContentsMargins(2, 2, 2, 2)

        label_font = QFont()
        bold_font = QFont()
        bold_font.setBold(True)

        def mk_lbl(text="", bold=False):
            lbl = QLabel(text)
            lbl.setFont(bold_font if bold else label_font)
            return lbl

        def group_box(title):
            g = QGroupBox(title)
            g.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid #888; border-radius: 2px; margin-top: 3px; } QGroupBox::title { subcontrol-origin: margin; left: 4px; padding: 0 2px; }")
            return g

        self._sys_g = group_box("System")
        sys_row = QHBoxLayout(self._sys_g)
        sys_row.setContentsMargins(2, 12, 2, 12)
        sys_row.addWidget(mk_lbl("State:", True))
        self._nav_state_label = mk_lbl("--", True)
        sys_row.addWidget(self._nav_state_label)
        sys_row.addWidget(mk_lbl("Batt:", True))
        self._nav_batt_volts = mk_lbl("--", True)
        sys_row.addWidget(self._nav_batt_volts)
        sys_row.addWidget(mk_lbl("mAh:", True))
        self._nav_batt_mah = mk_lbl("--", True)
        sys_row.addWidget(self._nav_batt_mah)
        sys_row.addWidget(mk_lbl("Curr:", True))
        self._nav_batt_current = mk_lbl("--", True)
        sys_row.addWidget(self._nav_batt_current)
        sys_row.addWidget(mk_lbl("Time:", True))
        self._nav_mission_time = mk_lbl("--", True)
        sys_row.addWidget(self._nav_mission_time)
        sys_row.addStretch()

        nav_g = group_box("Nav")
        nav_row = QHBoxLayout(nav_g)
        nav_row.setContentsMargins(2, 12, 2, 12)
        nav_row.addWidget(QLabel("WP:"))
        self._nav_curr_wp = mk_lbl("0")
        nav_row.addWidget(self._nav_curr_wp)
        nav_row.addWidget(QLabel("Dist:"))
        self._nav_wp_dist = mk_lbl("--")
        nav_row.addWidget(self._nav_wp_dist)
        nav_row.addWidget(QLabel("Brg:"))
        self._nav_wp_brg = mk_lbl("--")
        nav_row.addWidget(self._nav_wp_brg)
        nav_row.addWidget(QLabel("XTE:"))
        self._nav_wp_xte = mk_lbl("--")
        nav_row.addWidget(self._nav_wp_xte)
        nav_row.addWidget(QLabel("Trip:"))
        self._nav_dist_traveled = mk_lbl("0m")
        nav_row.addWidget(self._nav_dist_traveled)
        nav_row.addStretch()

        left_layout.addWidget(self._sys_g)
        left_layout.addWidget(nav_g)

        alt_g = group_box("Altitude")
        alt_row = QHBoxLayout(alt_g)
        alt_row.setContentsMargins(2, 12, 2, 12)
        alt_row.addWidget(QLabel("Press:"))
        self._nav_alt_press = mk_lbl("--")
        alt_row.addWidget(self._nav_alt_press)
        alt_row.addWidget(QLabel("KF:"))
        self._nav_alt_kf = mk_lbl("--")
        alt_row.addWidget(self._nav_alt_kf)
        alt_row.addWidget(QLabel("ROC:"))
        self._nav_alt_roc = mk_lbl("--")
        alt_row.addWidget(self._nav_alt_roc)
        alt_row.addWidget(QLabel("Des:"))
        self._nav_alt_desired = mk_lbl("--")
        alt_row.addWidget(self._nav_alt_desired)
        alt_row.addWidget(QLabel("GPS:"))
        self._nav_gps_alt = mk_lbl("--")
        alt_row.addWidget(self._nav_gps_alt)
        alt_row.addStretch()

        pos_g = group_box("Position")
        pos_row = QHBoxLayout(pos_g)
        pos_row.setContentsMargins(2, 12, 2, 12)
        pos_row.addWidget(QLabel("Lat:"))
        self._nav_gps_lat = mk_lbl("--")
        pos_row.addWidget(self._nav_gps_lat)
        pos_row.addWidget(QLabel("Lon:"))
        self._nav_gps_lon = mk_lbl("--")
        pos_row.addWidget(self._nav_gps_lon)
        pos_row.addWidget(QLabel("Vel:"))
        self._nav_gps_vel = mk_lbl("--")
        pos_row.addWidget(self._nav_gps_vel)
        pos_row.addWidget(QLabel("Sats:"))
        self._nav_gps_sats = mk_lbl("--")
        pos_row.addWidget(self._nav_gps_sats)
        pos_row.addStretch()

        left_layout.addWidget(alt_g)
        left_layout.addWidget(pos_g)
        left_layout.addStretch()

        h_splitter.addWidget(left_panel)

        # --- Right panel: waypoint table + buttons + home ---
        right_panel = QWidget()
        right_panel.setMinimumWidth(220)
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)

        self.wp_table = QTableWidget()
        self.wp_table.setColumnCount(7)
        self.wp_table.setHorizontalHeaderLabels(["#", "Lat", "Lon", "Alt", "V", "Loiter", "Action"])
        self.wp_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.wp_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.wp_table.setEditTriggers(QTableWidget.DoubleClicked | QTableWidget.EditKeyPressed)
        self.wp_table.itemChanged.connect(self._on_wp_cell_changed)
        right_layout.addWidget(self.wp_table)

        btn_layout = QHBoxLayout()
        self.add_via_btn = QPushButton("Add Via")
        btn_layout.addWidget(self.add_via_btn)
        self.add_orbit_btn = QPushButton("Add Orbit")
        btn_layout.addWidget(self.add_orbit_btn)
        self.add_poi_btn = QPushButton("Add POI")
        btn_layout.addWidget(self.add_poi_btn)
        self.add_land_btn = QPushButton("Add Land")
        btn_layout.addWidget(self.add_land_btn)
        self.del_wp_btn = QPushButton("Delete")
        self.del_wp_btn.setStyleSheet("color: red;")
        btn_layout.addWidget(self.del_wp_btn)
        btn_layout.addStretch()
        self.wp_count_label = QLabel("Waypoints: 0")
        btn_layout.addWidget(self.wp_count_label)
        right_layout.addLayout(btn_layout)

        home_layout = QHBoxLayout()
        home_layout.addWidget(QLabel("Home Lat:"))
        self.home_lat = QLineEdit("0.000000")
        self.home_lat.setMaximumWidth(120)
        home_layout.addWidget(self.home_lat)
        home_layout.addWidget(QLabel("Lon:"))
        self.home_lon = QLineEdit("0.000000")
        self.home_lon.setMaximumWidth(120)
        home_layout.addWidget(self.home_lon)
        self.set_home_btn = QPushButton("Set Home")
        home_layout.addWidget(self.set_home_btn)
        self.set_home_from_gps_btn = QPushButton("From GPS")
        home_layout.addWidget(self.set_home_from_gps_btn)
        self.set_home_from_center_btn = QPushButton("Center")
        home_layout.addWidget(self.set_home_from_center_btn)
        home_layout.addStretch()
        right_layout.addLayout(home_layout)

        h_splitter.addWidget(right_panel)
        h_splitter.setStretchFactor(0, 2)
        h_splitter.setStretchFactor(1, 1)

        bot_layout.addWidget(h_splitter)
        v_splitter.addWidget(bot_panel)
        v_splitter.setStretchFactor(0, 1)
        v_splitter.setStretchFactor(1, 0)

    def setup_connections(self):
        self.load_btn.clicked.connect(self.load_mission)
        self.save_btn.clicked.connect(self.save_mission)
        self.clear_btn.clicked.connect(self.clear_mission)
        self.read_btn.clicked.connect(self.read_from_uavx)
        self.write_btn.clicked.connect(self.write_to_uavx)
        self.add_via_btn.clicked.connect(lambda: self._select_wp_type("Via"))
        self.add_orbit_btn.clicked.connect(lambda: self._select_wp_type("Orbit"))
        self.add_poi_btn.clicked.connect(lambda: self._select_wp_type("POI"))
        self.add_land_btn.clicked.connect(lambda: self._select_wp_type("Land"))
        self.del_wp_btn.clicked.connect(self.delete_waypoint)
        self.set_home_btn.clicked.connect(self.set_home)
        self.set_home_from_gps_btn.clicked.connect(self.set_home_from_gps)
        self.set_home_from_center_btn.clicked.connect(self.set_home_from_center)
        self.follow_check.stateChanged.connect(self.update_map)
        self.zoom_in_btn.clicked.connect(lambda: self.zoom_map(1))
        self.zoom_out_btn.clicked.connect(lambda: self.zoom_map(-1))
        self.wp_table.itemSelectionChanged.connect(self.on_wp_selected)

    def load_default_mission(self):
        self.home_lat.setText("")
        self.home_lon.setText("")
        self.home_pos = None
        self.waypoints = []
        self.update_wp_table()

    def add_waypoint(self, action, lat=None, lon=None, alt=None, vel=None, loiter=None):
        if lat is None:
            self._pending_wp_action = action
            if WEBENGINE_AVAILABLE and FOLIUM_AVAILABLE:
                self.map_widget.page().runJavaScript(
                    "getMapCenter()",
                    self._on_add_wp_center
                )
            else:
                pos = self.home_pos or self.aircraft_pos
                if pos:
                    lat, lon = pos
                else:
                    lat, lon = 0, 0
                self._finish_add_waypoint(action, lat, lon, alt, vel, loiter)
            return
        self._finish_add_waypoint(action, lat, lon, alt, vel, loiter)

    def _on_add_wp_center(self, result):
        if isinstance(result, str) and ',' in result:
            parts = result.split(',')
            if len(parts) == 2:
                try:
                    lat = float(parts[0])
                    lon = float(parts[1])
                    self._finish_add_waypoint(self._pending_wp_action, lat, lon)
                    return
                except ValueError:
                    pass
        pos = self.home_pos or self.aircraft_pos
        if pos:
            lat, lon = pos
        else:
            lat, lon = 0, 0
        self._finish_add_waypoint(self._pending_wp_action, lat, lon)

    def _finish_add_waypoint(self, action, lat, lon, alt=None, vel=None, loiter=None):
        if alt is None:
            alt = 20
        if vel is None:
            vel = 3.0
        if loiter is None:
            loiter = 15

        wp = Waypoint(
            index=len(self.waypoints) + 1,
            lat=lat, lon=lon, alt=alt,
            velocity=vel, loiter=loiter, action=action,
        )
        self.waypoints.append(wp)
        self.update_wp_table()
        self._preserve_and_rebuild()

    def _preserve_and_rebuild(self):
        if WEBENGINE_AVAILABLE and FOLIUM_AVAILABLE:
            self.map_widget.page().runJavaScript(
                "getMapCenter()",
                self._on_do_rebuild
            )
        else:
            self.update_map()

    def _on_do_rebuild(self, result):
        if isinstance(result, str) and ',' in result:
            parts = result.split(',')
            if len(parts) >= 2:
                try:
                    lat, lon = float(parts[0]), float(parts[1])
                    self._override_center = (lat, lon)
                    self._override_zoom = int(float(parts[2])) if len(parts) >= 3 else self.map_zoom
                except ValueError:
                    pass
        self.update_map()

    def _select_wp_type(self, action):
        if self._active_wp_action == action:
            self._clear_wp_selection()
            return
        self._clear_wp_selection()
        self._active_wp_action = action
        btn = {"Via": self.add_via_btn, "Orbit": self.add_orbit_btn,
               "POI": self.add_poi_btn, "Land": self.add_land_btn}.get(action)
        if btn:
            btn.setStyleSheet("background-color: #4a90d9; color: white; font-weight: bold;")
        self.status_label.setText(f"Click map to place {action} at crosshairs")

    def _clear_wp_selection(self):
        self._active_wp_action = None
        for b in (self.add_via_btn, self.add_orbit_btn, self.add_poi_btn, self.add_land_btn):
            b.setStyleSheet("")
        self.status_label.setText("Ready")

    def _place_wp_from_center(self, result, action):
        if isinstance(result, str) and ',' in result:
            parts = result.split(',')
            if len(parts) >= 2:
                try:
                    lat, lon = float(parts[0]), float(parts[1])
                    self.add_waypoint(action, lat, lon)
                    self.status_label.setText(f"Placed {action} at {lat:.6f}, {lon:.6f}")
                    return
                except ValueError:
                    pass
        self.status_label.setText("Failed to place waypoint")

    def delete_waypoint(self):
        row = self.wp_table.currentRow()
        if row >= 0 and row < len(self.waypoints):
            self.waypoints.pop(row)
            self.update_wp_table()
            self._preserve_and_rebuild()

    def on_wp_selected(self):
        row = self.wp_table.currentRow()
        if row >= 0 and row < len(self.waypoints):
            wp = self.waypoints[row]
            self.status_label.setText(f"Selected WP {wp.index}: {wp.action} at ({wp.lat:.6f}, {wp.lon:.6f})")

    def _highlight_current_wp(self, wp_index):
        for row in range(self.wp_table.rowCount()):
            item = self.wp_table.item(row, 0)
            if item and int(item.text()) == wp_index:
                for col in range(self.wp_table.columnCount()):
                    c = self.wp_table.item(row, col)
                    if c:
                        c.setBackground(QColor(255, 255, 200))
            else:
                for col in range(self.wp_table.columnCount()):
                    c = self.wp_table.item(row, col)
                    if c:
                        c.setBackground(QColor(255, 255, 255))

    def update_wp_table(self):
        self.wp_table.blockSignals(True)
        self.wp_table.setRowCount(len(self.waypoints))
        for i, wp in enumerate(self.waypoints):
            item0 = QTableWidgetItem(str(i + 1))
            item0.setFlags(item0.flags() & ~Qt.ItemIsEditable)
            self.wp_table.setItem(i, 0, item0)
            self.wp_table.setItem(i, 1, QTableWidgetItem(f"{wp.lat:.6f}"))
            self.wp_table.setItem(i, 2, QTableWidgetItem(f"{wp.lon:.6f}"))
            self.wp_table.setItem(i, 3, QTableWidgetItem(str(wp.alt)))
            self.wp_table.setItem(i, 4, QTableWidgetItem(f"{wp.velocity:.1f}"))
            self.wp_table.setItem(i, 5, QTableWidgetItem(str(wp.loiter)))
            self.wp_table.setItem(i, 6, QTableWidgetItem(wp.action))
        self.wp_count_label.setText(f"Waypoints: {len(self.waypoints)}")
        self.wp_table.blockSignals(False)

    def _on_wp_cell_changed(self, item):
        row = item.row()
        col = item.column()
        if row >= len(self.waypoints):
            return
        wp = self.waypoints[row]
        try:
            if col == 1:
                wp.lat = float(item.text())
            elif col == 2:
                wp.lon = float(item.text())
            elif col == 3:
                wp.alt = int(float(item.text()))
            elif col == 4:
                wp.velocity = float(item.text())
            elif col == 5:
                wp.loiter = int(item.text())
            elif col == 6:
                wp.action = item.text()
        except ValueError:
            pass
        self._preserve_and_rebuild()

    def set_home(self):
        if WEBENGINE_AVAILABLE and FOLIUM_AVAILABLE:
            self.map_widget.page().runJavaScript(
                "getMapCenter()",
                self._on_set_home_js
            )
        else:
            self._set_home_from_fields()

    def _on_set_home_js(self, result):
        if isinstance(result, str) and ',' in result:
            parts = result.split(',')
            if len(parts) == 2:
                try:
                    lat = float(parts[0])
                    lon = float(parts[1])
                    self._apply_home(lat, lon)
                    return
                except ValueError:
                    pass
        self._set_home_from_fields()

    def _set_home_from_fields(self):
        try:
            lat = float(self.home_lat.text())
            lon = float(self.home_lon.text())
            self._apply_home(lat, lon)
        except ValueError:
            QMessageBox.warning(self, "Error", "Invalid coordinates")

    def _apply_home(self, lat, lon):
        self.home_lat.setText(f"{lat:.6f}")
        self.home_lon.setText(f"{lon:.6f}")
        self.home_pos = (lat, lon)
        self._map_rebuild_key = (self.aircraft_pos, len(self.waypoints), self.home_pos, self.map_zoom)
        if WEBENGINE_AVAILABLE and FOLIUM_AVAILABLE:
            self.map_widget.page().runJavaScript(f"""
                (function() {{
                    var d = document.querySelector('.folium-map');
                    if (!d) return;
                    var m = window[d.id];
                    if (!m) return;
                    if (window.homeMarker) {{
                        window.homeMarker.setLatLng([{lat}, {lon}]);
                        window.homeCircle.setLatLng([{lat}, {lon}]);
                    }} else {{
                        window.homeMarker = L.circleMarker([{lat}, {lon}], {{
                            radius: 10, color: 'green', fill: true,
                            fillColor: 'green', fillOpacity: 0.5
                        }}).addTo(m).bindPopup('Home');
                        window.homeCircle = L.circle([{lat}, {lon}], {{
                            radius: 50, color: 'green', fill: false,
                            weight: 1, opacity: 0.5
                        }}).addTo(m);
                    }}
                }})();
            """)
        self.status_label.setText(f"Home: {lat:.6f}, {lon:.6f}")

    def set_home_from_gps(self):
        if self.parent() and hasattr(self.parent(), 'flight_data'):
            flight_data = self.parent().flight_data
            if flight_data and flight_data.gps_lat != 0 and flight_data.gps_lon != 0:
                self._apply_home(flight_data.gps_lat, flight_data.gps_lon)
                return
        QMessageBox.warning(self, "Error", "No valid GPS data available")

    def set_home_from_center(self):
        self.set_home()

    def zoom_map(self, delta):
        self.map_zoom = max(3, min(20, self.map_zoom + delta))
        self.update_map()

    def update_from_telemetry(self):
        if self.parent() and hasattr(self.parent(), 'flight_data'):
            flight_data = self.parent().flight_data
            if flight_data:
                lat = getattr(flight_data, 'gps_lat', 0)
                lon = getattr(flight_data, 'gps_lon', 0)
                if lat != 0 and lon != 0:
                    if not self._position_ready:
                        self._on_position_ready(lat, lon)
                        return
                    self.update_aircraft_position(lat, lon)

                def set_lbl(attr, val, fmt="{:.1f}"):
                    lbl = getattr(self, attr, None)
                    if lbl is not None:
                        try:
                            v = float(val)
                            lbl.setText(fmt.format(v))
                        except (ValueError, TypeError):
                            lbl.setText(str(val))

                set_lbl("_nav_state_label", NAV_STATE_NAMES.get(getattr(flight_data, 'nav_state', 0), '--'))

                orange = len(flight_data.flag_bits) > 5 and flight_data.flag_bits[5]

                self._sys_g.setStyleSheet(
                    "QGroupBox { font-weight: bold; border: 1px solid orange; border-radius: 2px; margin-top: 3px; color: orange; } QGroupBox::title { subcontrol-origin: margin; left: 4px; padding: 0 2px; }"
                    if orange else
                    "QGroupBox { font-weight: bold; border: 1px solid #888; border-radius: 2px; margin-top: 3px; } QGroupBox::title { subcontrol-origin: margin; left: 4px; padding: 0 2px; }"
                )

                batt_v = getattr(flight_data, 'battery_volts', 0)
                if self._nav_batt_volts is not None:
                    self._nav_batt_volts.setText(f"{batt_v:.2f}")
                    self._nav_batt_volts.setStyleSheet("color: orange;" if orange else "")

                set_lbl("_nav_batt_current", getattr(flight_data, 'battery_current', 0), "{:.2f}")

                batt_mah = getattr(flight_data, 'battery_charge', 0)
                if self._nav_batt_mah is not None:
                    self._nav_batt_mah.setText(f"{batt_mah:.0f}")
                    self._nav_batt_mah.setStyleSheet("color: orange;" if orange else "")
                set_lbl("_nav_curr_wp", getattr(flight_data, 'curr_wp', 0), "{:.0f}")
                curr_wp = int(getattr(flight_data, 'curr_wp', 0))
                self._highlight_current_wp(curr_wp)
                set_lbl("_nav_wp_dist", getattr(flight_data, 'distance_to_wp', 0))
                set_lbl("_nav_wp_brg", getattr(flight_data, 'wp_bearing', 0))
                set_lbl("_nav_wp_xte", getattr(flight_data, 'cross_track_error', 0), "{:.1f}")
                set_lbl("_nav_gps_lat", getattr(flight_data, 'gps_lat', 0), "{:.6f}")
                set_lbl("_nav_gps_lon", getattr(flight_data, 'gps_lon', 0), "{:.6f}")
                set_lbl("_nav_gps_alt", getattr(flight_data, 'gps_altitude', 0))
                set_lbl("_nav_gps_vel", getattr(flight_data, 'gps_vel', 0))
                set_lbl("_nav_gps_sats", getattr(flight_data, 'gps_sats', 0), "{:.0f}")
                set_lbl("_nav_alt_press", getattr(flight_data, 'altitude', 0))
                set_lbl("_nav_alt_kf", getattr(flight_data, 'baro_altitude', 0))
                set_lbl("_nav_alt_roc", getattr(flight_data, 'roc', 0))
                set_lbl("_nav_alt_desired", getattr(flight_data, 'desired_altitude', 0))

                mt = getattr(flight_data, 'mission_time', 0)
                if self._nav_mission_time is not None:
                    if mt >= 3600:
                        self._nav_mission_time.setText(f"{int(mt//3600)}:{int((mt%3600)//60):02d}:{int(mt%60):02d}")
                    else:
                        self._nav_mission_time.setText(f"{int(mt//60):02d}:{int(mt%60):02d}")

                if lat != 0 and lon != 0:
                    if self._last_pos_for_dist is not None:
                        import math
                        dlat = math.radians(lat - self._last_pos_for_dist[0])
                        dlon = math.radians(lon - self._last_pos_for_dist[1])
                        a = math.sin(dlat/2)**2 + math.cos(math.radians(self._last_pos_for_dist[0])) * math.cos(math.radians(lat)) * math.sin(dlon/2)**2
                        self._distance_traveled += 2 * 6371000 * math.atan2(math.sqrt(a), math.sqrt(1-a))
                    self._last_pos_for_dist = (lat, lon)
                    if self._nav_dist_traveled is not None:
                        d = self._distance_traveled
                        if d < 1000:
                            self._nav_dist_traveled.setText(f"{d:.0f}m")
                        else:
                            self._nav_dist_traveled.setText(f"{d/1000:.2f}km")

    def _update_aircraft_on_map(self):
        if not self.aircraft_pos:
            return
        if not FOLIUM_AVAILABLE or not WEBENGINE_AVAILABLE:
            return
        key = self._map_rebuild_key
        if key is None:
            self.update_map()
            return
        # Full rebuild only if waypoints/home/zoom changed
        if len(self.waypoints) != key[1] or self.home_pos != key[2] or self.map_zoom != key[3]:
            self.update_map()
            return
        # Move aircraft marker via JS
        self.map_widget.page().runJavaScript(
            f"updateAircraftPosition({self.aircraft_pos[0]}, {self.aircraft_pos[1]})"
        )

    def update_map(self):
        if not FOLIUM_AVAILABLE:
            self.map_widget.setHtml("""
                <html><body style="font-family: Arial; text-align: center; padding: 50px;">
                    <h2>Map Not Available</h2>
                    <p>Folium is not available.</p>
                    <p>Please run: <b>pip install folium</b></p>
                    <p>Then restart the application.</p>
                </body></html>
            """)
            return

        try:
            if self._override_center:
                center = self._override_center
                z = self._override_zoom if self._override_zoom else self.map_zoom
            elif self.follow_check.isChecked() and self.aircraft_pos and self.home_pos:
                import math
                dlat = (self.aircraft_pos[0] - self.home_pos[0]) * 111000
                dlon = (self.aircraft_pos[1] - self.home_pos[1]) * 111000 * math.cos(math.radians(self.home_pos[0]))
                dist = math.sqrt(dlat*dlat + dlon*dlon)
                if dist < 5000:
                    center = self.aircraft_pos
                else:
                    center = self.home_pos
            elif self.aircraft_pos:
                center = self.aircraft_pos
            elif self.home_pos:
                center = self.home_pos
            else:
                center = (0, 0)

            m = folium.Map(
                location=center,
                zoom_start=z if self._override_zoom else self.map_zoom,
                tiles='OpenStreetMap'
            )

            wp_colors = {'Via': 'blue', 'Orbit': 'orange', 'POI': 'purple', 'Land': 'red'}

            for wp in self.waypoints:
                if wp.action == 'Orbit' and wp.loiter > 0:
                    folium.Circle(
                        (wp.lat, wp.lon),
                        radius=wp.loiter,
                        color='orange',
                        fill=False,
                        weight=1,
                        opacity=0.3
                    ).add_to(m)

            route_wps = [wp for wp in self.waypoints if wp.action != 'POI']
            if len(route_wps) > 1:
                points = [(wp.lat, wp.lon) for wp in route_wps]
                folium.PolyLine(
                    points,
                    color='yellow',
                    weight=3,
                    opacity=0.8,
                    popup=f"Mission Path ({len(points)} waypoints)"
                ).add_to(m)

            folium.LayerControl().add_to(m)

            # Click-to-add-WP + aircraft marker management
            map_js = """
            <script>
            window.aircraftMarker = null;

            function initAircraftMarker(lat, lon) {
                var mapDiv = document.querySelector('.folium-map');
                if (!mapDiv) return;
                var mapId = mapDiv.id;
                var map = window[mapId];
                if (!map) return;
                if (window.aircraftMarker) {
                    map.removeLayer(window.aircraftMarker);
                }
                window.aircraftMarker = L.circleMarker([lat, lon], {
                    radius: 6, color: 'red', fill: true, fillColor: 'red'
                }).addTo(map);
            }

            function updateAircraftPosition(lat, lon) {
                if (window.aircraftMarker) {
                    window.aircraftMarker.setLatLng([lat, lon]);
                } else {
                    initAircraftMarker(lat, lon);
                }
            }

            window.homeMarker = null;
            window.homeCircle = null;
            window.wpMarkers = [];

            function initWaypointMarkers(wps) {
                var mapDiv = document.querySelector('.folium-map');
                if (!mapDiv) return;
                var mapId = mapDiv.id;
                var map = window[mapId];
                if (!map) return;
                for (var i = 0; i < window.wpMarkers.length; i++) {
                    map.removeLayer(window.wpMarkers[i]);
                }
                window.wpMarkers = [];
                for (var i = 0; i < wps.length; i++) {
                    let wp = wps[i];
                    let m = L.circleMarker([wp.lat, wp.lon], {
                        radius: 7, color: wp.color, fill: true,
                        fillColor: wp.color, fillOpacity: 0.8
                    }).addTo(map).bindPopup(wp.popup);
                    m.on('mouseover', function() {
                        this.setStyle({radius: 10, fillOpacity: 1});
                    });
                    m.on('mouseout', function() {
                        this.setStyle({radius: 7, fillOpacity: 0.8});
                    });
                    let dragActive = false;
                    m.on('mousedown', function(e) {
                        map.dragging.disable();
                        dragActive = true;
                        let didMove = false;
                        var marker = this;
                        function onMove(me) {
                            if (!dragActive) return;
                            didMove = true;
                            var ll = map.mouseEventToLatLng(me.originalEvent);
                            marker.setLatLng(ll);
                        }
                        function onUp() {
                            if (!dragActive) return;
                            dragActive = false;
                            map.off('mousemove', onMove);
                            map.off('mouseup', onUp);
                            map.dragging.enable();
                            if (didMove) {
                                var ll = marker.getLatLng();
                                document.title = 'WPDRAG:' + wp.i + ',' + ll.lat.toFixed(7) + ',' + ll.lng.toFixed(7);
                            }
                            window._suppressClick = true;
                            setTimeout(function() { window._suppressClick = false; }, 0);
                        }
                        map.on('mousemove', onMove);
                        map.on('mouseup', onUp);
                    });
                    window.wpMarkers.push(m);
                }
            }

            function initHomeMarker(lat, lon) {
                var mapDiv = document.querySelector('.folium-map');
                if (!mapDiv) return;
                var mapId = mapDiv.id;
                var map = window[mapId];
                if (!map) return;
                if (window.homeMarker) {
                    map.removeLayer(window.homeMarker);
                }
                if (window.homeCircle) {
                    map.removeLayer(window.homeCircle);
                }
                window.homeMarker = L.circleMarker([lat, lon], {
                    radius: 10, color: 'green', fill: true, fillColor: 'green', fillOpacity: 0.5
                }).addTo(map).bindPopup('Home');
                window.homeCircle = L.circle([lat, lon], {
                    radius: 50, color: 'green', fill: false, weight: 1, opacity: 0.5
                }).addTo(map);
            }

            function updateHomePosition(lat, lon) {
                if (window.homeMarker) {
                    window.homeMarker.setLatLng([lat, lon]);
                    window.homeCircle.setLatLng([lat, lon]);
                } else {
                    initHomeMarker(lat, lon);
                }
            }

            function getMapCenter() {
                var mapDiv = document.querySelector('.folium-map');
                if (!mapDiv) return '0,0,10';
                var mapId = mapDiv.id;
                var map = window[mapId];
                if (!map) return '0,0,10';
                var c = map.getCenter();
                return c.lat.toFixed(7) + ',' + c.lng.toFixed(7) + ',' + map.getZoom();
            }

            document.addEventListener('DOMContentLoaded', function() {
                var mapDiv = document.querySelector('.folium-map');
                if (!mapDiv) return;
                var mapId = mapDiv.id;
                var map = window[mapId];
                if (!map) return;
                map.on('click', function(e) {
                    if (window._suppressClick) return;
                    var lat = e.latlng.lat.toFixed(7);
                    var lon = e.latlng.lng.toFixed(7);
                    document.title = 'WP:' + lat + ',' + lon;
                });
            });
            </script>
            """
            crosshair_html = """
            <style>
            .map-crosshair-line {
                position: fixed;
                background: rgba(255, 0, 0, 0.65);
                z-index: 10000;
                pointer-events: none;
            }
            .map-crosshair-h {
                top: 50%;
                left: 50%;
                width: 44px;
                height: 1.5px;
                transform: translate(-50%, -50%);
            }
            .map-crosshair-v {
                top: 50%;
                left: 50%;
                width: 1.5px;
                height: 44px;
                transform: translate(-50%, -50%);
            }
            .map-crosshair-dot {
                position: fixed;
                top: 50%;
                left: 50%;
                transform: translate(-50%, -50%);
                z-index: 10001;
                pointer-events: none;
                width: 5px;
                height: 5px;
                border-radius: 50%;
                background: rgba(255, 0, 0, 0.85);
                border: 1px solid rgba(255, 255, 255, 0.7);
            }
            </style>
            <div class="map-crosshair-line map-crosshair-h"></div>
            <div class="map-crosshair-line map-crosshair-v"></div>
            <div class="map-crosshair-dot"></div>
            """
            html = m.get_root().render()
            wp_data = json.dumps([{
                "i": wp.index, "lat": wp.lat, "lon": wp.lon,
                "color": wp_colors.get(wp.action, 'blue'),
                "popup": f"WP {wp.index}: {wp.action} Alt:{wp.alt}m Vel:{wp.velocity}m/s"
            } for wp in self.waypoints])
            wp_script = f'<script>window._wpData = {wp_data}</script>'
            html = html.replace('</body>', map_js + wp_script + crosshair_html + '</body>')
            self.map_widget.setHtml(html)
            self.map_widget.page().loadFinished.connect(self._on_map_loaded)

            self._map_rebuild_key = (self.aircraft_pos, len(self.waypoints), self.home_pos, self.map_zoom)

            wp_status = f"{len(self.waypoints)} waypoints"
            pos_status = ""
            if self.aircraft_pos:
                pos_status = f" | Aircraft: {self.aircraft_pos[0]:.6f}, {self.aircraft_pos[1]:.6f}"
            self.status_label.setText(f"Map updated - {wp_status}{pos_status}")

        except Exception as e:
            self.map_widget.setHtml(f"""
                <html><body style="font-family: Arial; text-align: center; padding: 50px; color: red;">
                    <h2>Map Error</h2>
                    <p>{str(e)}</p>
                    <p style="color: #666; font-size: 12px;">Check that folium is properly installed.</p>
                </body></html>
            """)
            print(f"Map error: {e}")

    def load_mission(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Mission", "",
            "Mission Files (*.txt *.json);;All Files (*.*)"
        )
        if path:
            try:
                with open(path, 'r') as f:
                    data = json.load(f)
                raw = data.get('waypoints', [])
                self.waypoints = [Waypoint(**wp) if isinstance(wp, dict) else wp for wp in raw]
                for i, wp in enumerate(self.waypoints):
                    wp.index = i + 1
                self.home_lat.setText(str(data.get('home_lat', 0)))
                self.home_lon.setText(str(data.get('home_lon', 0)))
                self.home_pos = (float(self.home_lat.text()), float(self.home_lon.text()))
                self.update_wp_table()
                self.update_map()
                self.status_label.setText(f"Loaded mission with {len(self.waypoints)} waypoints")
            except Exception as e:
                QMessageBox.warning(self, "Error", f"Failed to load mission: {str(e)}")

    def save_mission(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Mission",
            f"mission_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt",
            "Mission Files (*.txt *.json);;All Files (*.*)"
        )
        if path:
            try:
                data = {
                    'waypoints': [wp.to_dict() for wp in self.waypoints],
                    'home_lat': float(self.home_lat.text()),
                    'home_lon': float(self.home_lon.text()),
                    'timestamp': datetime.now().isoformat()
                }
                with open(path, 'w') as f:
                    json.dump(data, f, indent=2)
                self.status_label.setText(f"Saved mission to {os.path.basename(path)}")
            except Exception as e:
                QMessageBox.warning(self, "Error", f"Failed to save mission: {str(e)}")

    def clear_mission(self):
        if self.waypoints:
            reply = QMessageBox.question(
                self, "Clear Mission",
                "Delete all waypoints?",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply == QMessageBox.No:
                return
        self.waypoints = []
        self.update_wp_table()
        self.update_map()
        self.status_label.setText("Mission cleared")

    def read_from_uavx(self):
        parent = self.parent()
        if not parent or not hasattr(parent, 'send_request'):
            QMessageBox.warning(self, "Error", "Not connected")
            return

        self._read_queue = []
        self._read_pending = False
        self.waypoints.clear()
        parent.origin_data = None
        self._origin_retries = 0
        self.status_label.setText("Requesting origin from UAVX...")
        parent.send_request(PacketTag.ORIGIN, 0xFF, 0)
        QTimer.singleShot(200, self._poll_read_origin)

    def _poll_read_origin(self):
        parent = self.parent()
        if not parent:
            return
        origin = getattr(parent, 'origin_data', None)
        if origin and isinstance(origin, OriginData) and origin.num_waypoints > 0:
            self.home_pos = (origin.home_lat * 1e-7, origin.home_lon * 1e-7)
            self.home_lat.setText(f"{origin.home_lat * 1e-7:.6f}")
            self.home_lon.setText(f"{origin.home_lon * 1e-7:.6f}")
            n = origin.num_waypoints
            self.status_label.setText(f"Reading {n} waypoints...")
            self._read_queue = list(range(n))
            self._read_next_wp()
        elif origin and isinstance(origin, OriginData):
            self.status_label.setText("No waypoints on FC")
            self.update_wp_table()
            self.update_map()
        else:
            self._origin_retries += 1
            if self._origin_retries > 25:
                self.status_label.setText("Timeout: no response from FC")
                return
            QTimer.singleShot(200, self._poll_read_origin)

    def _read_next_wp(self):
        if not self._read_queue:
            self._finish_read()
            return
        idx = self._read_queue.pop(0)
        self.status_label.setText(f"Reading WP {idx + 1}...")
        parent = self.parent()
        if parent and hasattr(parent, 'send_request'):
            parent._last_wp_data = None
            parent.send_request(PacketTag.WP, idx, 0)
        self._wp_retries = 0
        QTimer.singleShot(300, lambda: self._poll_read_wp(idx))

    def _poll_read_wp(self, expected_idx):
        parent = self.parent()
        if not parent:
            return
        wp_data = getattr(parent, '_last_wp_data', None)
        if wp_data and wp_data.get('wp_index') == expected_idx:
            wp = Waypoint.from_packet(wp_data)
            wp.index = expected_idx + 1
            self.waypoints.append(wp)
            self._read_next_wp()
        else:
            self._wp_retries += 1
            if self._wp_retries > 15:
                self._finish_read()
                return
            QTimer.singleShot(200, lambda: self._poll_read_wp(expected_idx))

    def _finish_read(self):
        self.update_wp_table()
        self.update_map()
        self.status_label.setText(f"Read {len(self.waypoints)} waypoints from UAVX")

    def write_to_uavx(self):
        parent = self.parent()
        if not parent or not hasattr(parent, 'send_raw_packet'):
            QMessageBox.warning(self, "Error", "Not connected")
            return

        n = len(self.waypoints)
        self.status_label.setText(f"Writing {n} waypoints to UAVX...")

        try:
            home_lat = int(float(self.home_lat.text()) * 1e7)
            home_lon = int(float(self.home_lon.text()) * 1e7)
        except ValueError:
            home_lat = home_lon = 0

        # Send WP packets first, then origin last.
        # ProcessOriginPacket calls UpdateNavMission() which copies
        # NewNavMission (including WPs written above) to Config.Mission.
        for wp in self.waypoints:
            action_val = WP_ACTION_VALUES.get(wp.action, 0)
            body = struct.pack(
                "<BiiHHHHHHiib",
                wp.index - 1 if wp.index > 0 else 0,
                int(wp.lat * 1e7),
                int(wp.lon * 1e7),
                int(wp.alt),
                int(wp.velocity * 10),
                int(wp.loiter),
                0, 0, 0, 0, 0,
                action_val,
            )
            parent.send_raw_packet(PacketTag.WP, body)
            QThread.msleep(30)

        origin_body = struct.pack(
            "<Bbhhii",
            n if n < 128 else 0,
            50,
            100,
            0,
            home_lat,
            home_lon,
        )
        parent.send_raw_packet(PacketTag.ORIGIN, origin_body)
        QThread.msleep(50)

        self.status_label.setText(f"Written {n} waypoints to UAVX")

    def update_aircraft_position(self, lat, lon):
        old_pos = self.aircraft_pos
        self.aircraft_pos = (lat, lon)
        if not FOLIUM_AVAILABLE or not WEBENGINE_AVAILABLE:
            return
        if old_pos is None:
            self.map_widget.page().runJavaScript(
                f"initAircraftMarker({lat}, {lon})"
            )
            return
        self.map_widget.page().runJavaScript(
            f"updateAircraftPosition({lat}, {lon})"
        )

    def closeEvent(self, event):
        if self.telemetry_timer:
            self.telemetry_timer.stop()
        if self._map_update_timer:
            self._map_update_timer.stop()
        event.accept()
