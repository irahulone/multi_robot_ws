import sys
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int16, String, Float32MultiArray

from .my_ros_module import JoyBase

# PyQt6のインポート
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QWidget,
    QSpinBox, QComboBox, QCheckBox
)
from PyQt6.QtCore import QTimer


class JoyCmd(JoyBase):
    def __init__(self):
        super().__init__('joy_cmd')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('lx', "LY"),
                ('az', "RX"),
                ('en', "LB"),
                ('rover_sel', "Y"),
                ('mode_sel', "X"),
                ('angle_sel', "A"),
                ('hardware_sim_sel', "B"),
                ('broadcast', "RB"),
                ('revolution', "cross_lr"),
                ('prismatic', "cross_ud"),
                ('n_rover', 6),
            ]
        )

        self.lx_axisN = self.axis_dict.get(self.get_parameter('lx').value)
        self.az_axisN = self.axis_dict.get(self.get_parameter('az').value)
        self.en_buttonN = self.button_dict.get(self.get_parameter('en').value)
        self.rover_sel_button = self.button_dict.get(self.get_parameter('rover_sel').value)
        self.mode_sel_button = self.button_dict.get(self.get_parameter('mode_sel').value)
        self.angle_sel_button = self.button_dict.get(self.get_parameter('angle_sel').value)
        self.hw_sel_button = self.button_dict.get(self.get_parameter('hardware_sim_sel').value)
        self.broadcast_button = self.button_dict.get(self.get_parameter('broadcast').value)
        self.N_ROVER = self.get_parameter('n_rover').value

        self.mode_list = ["NEU_M", "JOY_M", "NAV_M"]
        self.mode_dict = {"NEU_M": 0, "JOY_M": 1, "NAV_M": 2}
        self.rover_modeC = self.mode_list[0]

        self.select = 1
        self.selected_angle = 0
        self.hw_sel = True

        self.pubsub.create_publisher(Twist, '/joy/cmd_vel', 5)
        self.pubsub.create_publisher(Bool, '/joy/enable', 5)
        self.pubsub.create_publisher(Bool, '/joy/broadcast', 1)
        self.pubsub.create_publisher(Bool, '/joy/hardware', 1)
        self.pubsub.create_publisher(Int16, '/select_rover', 1)
        self.pubsub.create_publisher(String, '/modeC', 1)
        self.pubsub.create_publisher(Int16, '/joy/angle_sel', 5)
        self.pubsub.create_publisher(Float32MultiArray, '/joy/cross', 5)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(
            f"JoyCmd Node Initialized\n"
            f"lx: {self.get_parameter('lx').value},\n"
            f"az: {self.get_parameter('az').value},\n"
            f"en: {self.get_parameter('en').value},\n"
            f"rover_sel: {self.get_parameter('rover_sel').value},\n"
            f"mode_sel: {self.get_parameter('mode_sel').value},\n"
            f"angle_sel: {self.get_parameter('angle_sel').value},\n"
            f"hardware_sel: {self.get_parameter('hardware_sim_sel').value},\n"
            f"broadcast: {self.get_parameter('broadcast').value},\n"
            f"revolution: {self.get_parameter('revolution').value},\n"
            f"prismatic: {self.get_parameter('prismatic').value}\n"
        )

    def joy_callback(self, msg):
        _toggle = self.joy_toggle(msg)
        j_lx = msg.axes[self.lx_axisN]
        j_az = msg.axes[self.az_axisN]

        val = Twist()         # 選択されたローバーへ送信するTwistメッセージ
        en_state = Bool()     # 選択されたローバーの有効状態
        empty_twist = Twist() # その他のローバー用（ゼロ値）
        false_state = Bool()  # その他のローバー用（False状態）
        
        empty_twist.linear.x = 0.0
        empty_twist.angular.z = 0.0
        false_state.data = False

        # ローバー選択トグル
        if _toggle[self.rover_sel_button] == 1:
            self.select = self.select % self.N_ROVER + 1
            self.get_logger().info(f"Sel Button Toggled: {self.select}")

        # HW/Simトグル
        if _toggle[self.hw_sel_button] == 1:
            self.hw_sel = not self.hw_sel
            self.get_logger().info(f"HW/Sim Button Toggled: {'HW' if self.hw_sel else 'Sim'}")

        # モード選択トグル
        if _toggle[self.mode_sel_button] == 1:
            new_mode = self.mode_list[(self.mode_dict[self.rover_modeC] + 1) % 3]
            self.get_logger().info(f"Mode Button Toggled: {self.rover_modeC} to {new_mode}")
            self.rover_modeC = new_mode
        
        # 角度選択トグル
        if _toggle[self.angle_sel_button] == 1:
            new_angle = (self.selected_angle + 90) % 360
            self.get_logger().info(f"Angle Button Toggled: {self.selected_angle} to {new_angle}")
            self.selected_angle = new_angle
            angle_msg = Int16()
            angle_msg.data = self.selected_angle
            self.pubsub.publish('/joy/angle_sel', angle_msg)
            
        broadcast_msg = Bool()
        broadcast_msg.data = True if msg.buttons[self.broadcast_button] == 1 else False
        self.pubsub.publish('/joy/broadcast', broadcast_msg)

        if msg.buttons[self.en_buttonN] == 1:
            en_state.data = True
            val.linear.x = 0.7 * j_lx
            val.angular.z = 0.5 * j_az
            self.pubsub.publish('/joy/cmd_vel', val)
            self.pubsub.publish('/joy/enable', en_state)
        else:
            self.pubsub.publish('/joy/cmd_vel', empty_twist)
            self.pubsub.publish('/joy/enable', false_state)
        
        msg_formation = Float32MultiArray()
        prismatic_val = msg.axes[self.axis_dict[self.get_parameter('prismatic').value]]
        revolution_val = msg.axes[self.axis_dict[self.get_parameter('revolution').value]]
        msg_formation.data = [prismatic_val, revolution_val]
        self.pubsub.publish('/joy/cross', msg_formation)

    def timer_callback(self):
        select_msg = Int16()
        select_msg.data = self.select
        self.pubsub.publish('/select_rover', select_msg)

        mode_msg = String()
        mode_msg.data = self.rover_modeC
        self.pubsub.publish('/modeC', mode_msg)   

        hardware_msg = Bool()
        hardware_msg.data = self.hw_sel
        self.pubsub.publish('/joy/hardware', hardware_msg)


class StatusWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("JoyCmd Status")

        central_widget = QWidget()
        # メインレイアウトは左右に分割する（QHBoxLayout）
        main_layout = QHBoxLayout()

        # 左側レイアウト：状態表示と変更ウィジェット
        left_layout = QVBoxLayout()
        # ステータス表示セクション
        status_header = QLabel("Status")
        left_layout.addWidget(status_header)
        self.status_labels = {
            "selected_rover": QLabel("Selected Rover: N/A"),
            "mode": QLabel("Mode: N/A"),
            "selected_angle": QLabel("Selected Angle: N/A"),
            "hardware": QLabel("Hardware: N/A"),
        }
        for label in self.status_labels.values():
            left_layout.addWidget(label)

        # ステータス変更用セクション
        modify_header = QLabel("Modify Status")
        left_layout.addWidget(modify_header)

        # 選択ローバーの変更（QSpinBox）
        self.rover_spin = QSpinBox()
        self.rover_spin.setMinimum(1)
        self.rover_spin.setMaximum(self.node.N_ROVER)
        self.rover_spin.setValue(self.node.select)
        self.rover_spin.valueChanged.connect(self.update_rover)
        left_layout.addWidget(QLabel("Select Rover:"))
        left_layout.addWidget(self.rover_spin)

        # モードの変更（QComboBox）
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(self.node.mode_list)
        current_mode_index = self.node.mode_dict[self.node.rover_modeC]
        self.mode_combo.setCurrentIndex(current_mode_index)
        self.mode_combo.currentTextChanged.connect(self.update_mode)
        left_layout.addWidget(QLabel("Mode:"))
        left_layout.addWidget(self.mode_combo)

        # 角度の変更（QComboBox、90度刻み）
        self.angle_combo = QComboBox()
        angles = ["0", "90", "180", "270"]
        self.angle_combo.addItems(angles)
        self.angle_combo.setCurrentIndex(self.node.selected_angle // 90)
        self.angle_combo.currentTextChanged.connect(self.update_angle)
        left_layout.addWidget(QLabel("Selected Angle:"))
        left_layout.addWidget(self.angle_combo)

        # HW/Simの変更（QCheckBox）
        self.hw_checkbox = QCheckBox("HW Mode")
        self.hw_checkbox.setChecked(self.node.hw_sel)
        self.hw_checkbox.toggled.connect(self.update_hw)
        left_layout.addWidget(self.hw_checkbox)

        # 右側レイアウト：ボタンマッピング表示
        right_layout = QVBoxLayout()
        mapping_header = QLabel("Button Assignments")
        right_layout.addWidget(mapping_header)
        mapping = {
            "forward/backward vel": self.node.get_parameter('lx').value,
            "rotation vel": self.node.get_parameter('az').value,
            "enable": self.node.get_parameter('en').value,
            "rover_sel": self.node.get_parameter('rover_sel').value,
            "mode": self.node.get_parameter('mode_sel').value,
            "angle_sel": self.node.get_parameter('angle_sel').value,
            "hardware/sim": self.node.get_parameter('hardware_sim_sel').value,
            "broadcast": self.node.get_parameter('broadcast').value,
        }
        self.mapping_labels = {}
        for key, value in mapping.items():
            label = QLabel(f"{key}: {value}")
            self.mapping_labels[key] = label
            right_layout.addWidget(label)

        # メインレイアウトに左側・右側レイアウトを追加
        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout)

        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        # GUI状態更新用タイマー（100msごと）
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_status)
        self.update_timer.start(100)

    def update_status(self):
        self.status_labels["selected_rover"].setText(f"Selected Rover: {self.node.select}")
        self.status_labels["mode"].setText(f"Mode: {self.node.rover_modeC}")
        self.status_labels["selected_angle"].setText(f"Selected Angle: {self.node.selected_angle}")
        hardware_str = "HW" if self.node.hw_sel else "Sim"
        self.status_labels["hardware"].setText(f"Hardware: {hardware_str}")

    def update_rover(self, value):
        self.node.select = value

    def update_mode(self, text):
        self.node.rover_modeC = text

    def update_angle(self, text):
        try:
            angle = int(text)
            self.node.selected_angle = angle
            angle_msg = Int16()
            angle_msg.data = angle
            self.node.pubsub.publish('/joy/angle_sel', angle_msg)
        except ValueError:
            pass

    def update_hw(self, checked):
        self.node.hw_sel = checked


def main(args=None):
    rclpy.init(args=args)
    node = JoyCmd()

    app = QApplication(sys.argv)
    gui = StatusWindow(node)
    gui.show()

    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    ros_timer.start(10)

    exit_code = app.exec()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
