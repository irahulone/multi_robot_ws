import sys
import rclpy
import json
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from PyQt5.QtCore import (
    Qt, QSize, pyqtSignal, QObject
)
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem,
    QDialog, QLabel, QLineEdit, QPushButton, QMessageBox, QFormLayout,
    QHeaderView
)
from PyQt5.QtGui import QFontMetrics
from rosidl_runtime_py.utilities import get_message
from threading import Thread, Event
from rclpy.parameter import Parameter
from builtin_interfaces.msg import Time, Duration


class ROS2JSONEncoder(json.JSONEncoder):
    """ ROS2 メッセージ用のカスタム JSON エンコーダ """
    def default(self, obj):
        try:
            if isinstance(obj, np.ndarray):
                return obj.tolist()  # NumPy 配列をリストに変換
            elif isinstance(obj, Parameter):
                return {"name": obj.name, "type": obj.type_, "value": obj.value}
            elif isinstance(obj, (Time, Duration)):
                return {"secs": obj.sec, "nanosecs": obj.nanosec}
            elif hasattr(obj, "get_fields_and_field_types"):  # ROS2 メッセージ型
                return ROS2GUI.message_to_dict(obj)
        except Exception as e:
            print(f"JSON serialization error: {e}")
        return str(obj)  # 未知のオブジェクトは文字列として処理


class UpdateSignal(QObject):
    """
    他スレッドからのGUI更新を受け付けるためのSignal。
    topic_name(str), formatted_msg(str)を受け取ってスレッドセーフにGUI更新へ渡す。
    """
    new_message = pyqtSignal(str, str)


class RecursiveInputDialog(QDialog):
    """ 任意のROS2メッセージ型に対応するダイアログ（再帰的にネストを処理） """

    def __init__(self, topic, msg_type, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"Publish Message to {topic}")
        self.msg_type = msg_type
        self.fields = {}

        layout = QVBoxLayout()
        form_layout = QFormLayout()

        self.fields = self.create_fields(msg_type, form_layout)
        layout.addLayout(form_layout)

        self.ok_button = QPushButton("Publish")
        self.ok_button.clicked.connect(self.on_ok_clicked)
        layout.addWidget(self.ok_button)

        self.setLayout(layout)

    def create_fields(self, msg_type, layout, prefix=""):
        """ 再帰的にフィールドを作成 """
        fields = {}
        for field_name, field_type in msg_type.get_fields_and_field_types().items():
            full_name = f"{prefix}{field_name}" if prefix else field_name
            # ネスト先の型を確認
            ros_type = None
            if "/" in field_type:
                try:
                    ros_type = get_message(field_type)
                except (AttributeError, ValueError):
                    ros_type = None

            if ros_type is not None:
                # ネストされたROS2メッセージの場合
                sub_form_layout = QFormLayout()
                sub_fields = self.create_fields(ros_type, sub_form_layout, full_name + ".")
                layout.addRow(QLabel(f"{full_name}:"), sub_form_layout)
                fields.update(sub_fields)
            else:
                # 配列型かどうかを判定
                if "[" in field_type and "]" in field_type:
                    # 例: float64[] / float64[4] / int32[]
                    fields[full_name] = QLineEdit()
                    layout.addRow(f"{full_name} (comma-separated array of {field_type}):", fields[full_name])
                else:
                    # 基本型（int, float, str, bool など）
                    fields[full_name] = QLineEdit()
                    layout.addRow(f"{full_name} ({field_type}):", fields[full_name])

        return fields

    def on_ok_clicked(self):
        # バリデーションやエラー発生時の制御をここで行う
        msg_instance = self.get_values()
        if msg_instance is not None:
            self.accept()  # OKとみなし、ダイアログを閉じる

    def get_values(self):
        """ 入力された値を取得してROS2メッセージを作成 """
        msg_instance = self.msg_type()
        for field_name, field_widget in self.fields.items():
            field_value = field_widget.text().strip()  # ユーザーが入力した値
            field_parts = field_name.split(".")
            sub_msg = msg_instance

            try:
                for part in field_parts[:-1]:  # ネストされたフィールドにアクセス
                    sub_msg = getattr(sub_msg, part)

                final_field = field_parts[-1]
                field_data = getattr(sub_msg, final_field)

                # 型情報を動的に取得して変換を行う
                # まずはPythonの型に応じて処理する
                if isinstance(field_data, np.ndarray):
                    # NumPy配列の場合
                    if not field_value:
                        # 空なら全て0で初期化（サイズはそのまま）
                        field_value = np.zeros_like(field_data)
                    else:
                        # CSVをパースしてリスト化
                        parsed = [float(v) for v in field_value.split(",")]
                        # サイズ固定の場合は要注意(超過や不足の扱い)
                        if field_data.size == len(parsed):
                            field_value = np.array(parsed, dtype=field_data.dtype)
                        else:
                            # 可変長ならそのままOK、固定長ならエラー
                            try:
                                field_value = np.array(parsed, dtype=np.float64)
                            except ValueError as e:
                                QMessageBox.warning(self, "Error",
                                                    f"Invalid input (array) for {field_name}:\n{str(e)}")
                                return None

                    setattr(sub_msg, final_field, field_value)
                else:
                    # 配列型(string[], float64[], int32[])など
                    # ROS2でPythonバインディングされるときはlistになることが多い
                    if isinstance(field_data, list):
                        if not field_value:
                            # 空入力なら空配列やデフォルト値
                            setattr(sub_msg, final_field, [])
                        else:
                            # CSVをパース
                            # 本来は型判定して変換する(ここではfloatで例示)
                            try:
                                parsed = [float(v) for v in field_value.split(",")]
                                setattr(sub_msg, final_field, parsed)
                            except ValueError:
                                # 文字列配列、int配列など分けて実装するのが望ましい
                                # とりあえず失敗したら警告
                                QMessageBox.warning(
                                    self, "Error",
                                    f"Failed to parse array for {field_name}. Check input format."
                                )
                                return None
                    else:
                        # 単一要素(int, float, bool, str, etc.)
                        if not field_value:
                            # 空欄ならデフォルト値を適用
                            if isinstance(field_data, bool):
                                field_value_converted = False
                            elif isinstance(field_data, int):
                                field_value_converted = 0
                            elif isinstance(field_data, float):
                                field_value_converted = 0.0
                            elif isinstance(field_data, str):
                                field_value_converted = ""
                            else:
                                field_value_converted = field_data
                        else:
                            # 型変換（int, float, bool, etc.）
                            if isinstance(field_data, bool):
                                # bool判定(簡易実装)
                                lower_val = field_value.lower()
                                if lower_val in ["true", "1"]:
                                    field_value_converted = True
                                else:
                                    field_value_converted = False
                            elif isinstance(field_data, int):
                                field_value_converted = int(field_value)
                            elif isinstance(field_data, float):
                                field_value_converted = float(field_value)
                            else:
                                # string など
                                field_value_converted = field_value

                        setattr(sub_msg, final_field, field_value_converted)

            except ValueError as e:
                QMessageBox.warning(self, "Error",
                                    f"Invalid input for {field_name}: {field_value}\n{str(e)}")
                return None
            except AttributeError as e:
                QMessageBox.warning(self, "Error",
                                    f"Attribute error for {field_name}:\n{str(e)}")
                return None

        return msg_instance


class ROS2TopicMonitor(Node):
    def __init__(self, update_signal):
        super().__init__('ros2_topic_monitor')
        self.update_signal = update_signal
        self.subscribers = {}
        self.topic_publishers = {}
        self.topic_types = {}

        # 定期的にトピックを検索
        self.create_timer(1.0, self.update_topics)

    def update_topics(self):
        topic_names_and_types = self.get_topic_names_and_types()
        for topic, types in topic_names_and_types:
            if topic not in self.subscribers and types:
                msg_type = None
                try:
                    msg_type = get_message(types[0])
                except (AttributeError, ValueError):
                    # 取得できない型の場合は無視
                    continue

                self.topic_types[topic] = msg_type
                self.create_subscriber(topic, msg_type)
                self.register_publisher(topic, msg_type)

    def create_subscriber(self, topic_name, msg_type):
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        def callback(msg):
            # ここはROSコールバック(別スレッド)
            formatted_msg = json.dumps(
                ROS2GUI.message_to_dict(msg),
                indent=2,
                cls=ROS2JSONEncoder
            )
            # Signalを使ってメインスレッドで更新
            self.update_signal.new_message.emit(topic_name, formatted_msg)

        self.subscribers[topic_name] = self.create_subscription(msg_type, topic_name, callback, qos)

    def register_publisher(self, topic_name, msg_type):
        if topic_name not in self.topic_publishers:
            self.topic_publishers[topic_name] = self.create_publisher(msg_type, topic_name, 10)

    def publish_message(self, topic_name, msg_instance):
        if topic_name in self.topic_publishers:
            try:
                self.topic_publishers[topic_name].publish(msg_instance)
            except Exception as e:
                # QMessageBoxを使うときはGUIスレッドで行う必要がある
                print(f"Failed to publish message: {e}")


class ROS2GUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Topic Monitor")
        self.setGeometry(100, 100, 800, 600)

        self.layout = QVBoxLayout()
        self.table = QTableWidget()
        self.table.setColumnCount(2)
        self.table.setHorizontalHeaderLabels(["Topic", "Latest Message"])
        self.layout.addWidget(self.table)
        self.setLayout(self.layout)

        self.table.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(
            1, QHeaderView.Stretch)
        self.table.setWordWrap(True)

        # Signalオブジェクトを作成
        self.update_signal = UpdateSignal()
        # Signalとスロット(関数)を接続
        self.update_signal.new_message.connect(self.update_table)

        # ROS Nodeを起動
        self.node = ROS2TopicMonitor(self.update_signal)
        self.ros_thread_stop_event = Event()
        self.ros_thread = Thread(target=self.run_ros_spin, daemon=True)
        self.ros_thread.start()

        self.table.cellDoubleClicked.connect(self.send_message_popup)

    def run_ros_spin(self):
        rclpy.spin(self.node)

    def closeEvent(self, event):
        """ウィンドウを閉じる時に呼ばれる"""
        # ROSスピンを安全に停止
        self.node.destroy_node()
        rclpy.shutdown()
        self.ros_thread_stop_event.set()
        super().closeEvent(event)

    def update_table(self, topic, msg):
        """ メインスレッドで呼ばれるスロット。最新のメッセージをテーブルに反映 """
        row = self.get_row_for_topic(topic)
        self.table.setItem(row, 0, QTableWidgetItem(topic))

        item = QTableWidgetItem(msg)
        item.setTextAlignment(Qt.AlignTop)
        item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)  # 編集不可

        font_metrics = QFontMetrics(self.table.font())
        text_height = font_metrics.boundingRect(
            0, 0,
            self.table.columnWidth(1),
            0,
            Qt.TextWordWrap,
            msg
        ).height()
        item.setSizeHint(QSize(self.table.columnWidth(1), text_height + 10))
        self.table.setItem(row, 1, item)
        self.table.resizeRowsToContents()

    def get_row_for_topic(self, topic):
        """ トピックごとの行を取得（なければ新しい行を追加） """
        for row in range(self.table.rowCount()):
            if self.table.item(row, 0) and self.table.item(row, 0).text() == topic:
                return row
        self.table.insertRow(self.table.rowCount())
        return self.table.rowCount() - 1

    def send_message_popup(self, row, column):
        """ ダブルクリックでメッセージ送信ポップアップを開く """
        topic_item = self.table.item(row, 0)
        if not topic_item:
            return
        topic = topic_item.text()
        msg_type = self.node.topic_types.get(topic, None)

        if msg_type is None:
            QMessageBox.warning(self, "Error", "Message type not found for this topic.")
            return

        dialog = RecursiveInputDialog(topic, msg_type, self)
        if dialog.exec_() == QDialog.Accepted:
            msg_instance = dialog.get_values()
            if msg_instance:
                self.node.publish_message(topic, msg_instance)

    @staticmethod
    def message_to_dict(msg):
        """ ROS2 メッセージを辞書形式に変換（再帰処理） """
        msg_dict = {}
        try:
            for field_name in msg.get_fields_and_field_types().keys():
                value = getattr(msg, field_name)

                if hasattr(value, "get_fields_and_field_types"):
                    msg_dict[field_name] = ROS2GUI.message_to_dict(value)
                elif isinstance(value, np.ndarray):
                    msg_dict[field_name] = value.tolist()
                elif isinstance(value, Parameter):
                    msg_dict[field_name] = {"name": value.name, "type": value.type_, "value": value.value}
                elif isinstance(value, (Time, Duration)):
                    msg_dict[field_name] = {"secs": value.sec, "nanosecs": value.nanosec}
                else:
                    msg_dict[field_name] = value
        except Exception as e:
            print(f"Error parsing ROS2 message: {e}")
            msg_dict["error"] = str(e)
        return msg_dict


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gui = ROS2GUI()
    gui.show()
    exit_code = app.exec_()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
