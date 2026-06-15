import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class SafetyNode(Node):
	def __init__(self):
		super().__init__("safety_node")

		self.declare_parameter("frame_id", "real_data")
		self.declare_parameter("odd_topic", "/visualize_real_data/odd_detections")
		self.declare_parameter("marker_topic", "/visualize_real_data/camera_polygons")
		self.declare_parameter("marker_ns", "camera_safety")
		self.declare_parameter("line_width", 0.1)
		self.declare_parameter("frame_position", '{"x": 15.35, "y": 5.7, "z": -0.2}')

		# Store frame position offset
		self.frame_position = json.loads(self.get_parameter("frame_position").get_parameter_value().string_value)

		# Camera map: camera -> polygon points.
		# Each point can be either {"x": ..., "y": ..., "z": ...}
		# or [x, y, z].
		self.camera_polygons = {
			"160": [
				{"x": 29.589738845825195, "y": 1.8118257522583008},
				{"x": 25.60017967224121, "y": -1.5746631622314453},
				{"x": 17.847824096679688, "y": 5.324751853942871},
				{"x": 22.30218505859375, "y": 10.756917953491211}
			],
			"161": [
				{"x": 28.85472869873047, "y": 1.336782455444336},
				{"x": 21.560569763183594, "y": 3.0320091247558594},
				{"x": 21.098522186279297, "y": 12.288042068481445},
				{"x": 27.11311912536621, "y": 14.379698753356934}
			],
			"162": [
				{"x": 23.786651611328125, "y": 2.886286735534668},
				{"x": 10.604456901550293, "y": -0.8180313110351562},
				{"x": 10.05613899230957, "y": 8.454113960266113},
				{"x": 23.24528694152832, "y": 7.421191215515137}
			],
			"163": [
				{"x": 11.437211036682129, "y": 0.6589336395263672},
				{"x": -0.7807378768920898, "y": 2.39864444732666},
				{"x": -0.8571281433105469, "y": 7.100903511047363},
				{"x": 11.596370697021484, "y": 9.124549865722656}
			],
			"164": [
				{"x": 28.099159240722656, "y": -2.012518882751465},
				{"x": 18.40500831604004, "y": -2.610698699951172},
				{"x": 16.493038177490234, "y": 3.397061347961426},
				{"x": 29.30846405029297, "y": 5.209956169128418}
			],
			"165": [
				{"x": 32.617835998535156, "y": -1.8187446594238281},
				{"x": 25.934892654418945, "y": -1.9089021682739258},
				{"x": 23.09740447998047, "y": 4.571246147155762},
				{"x": 34.130191802978516, "y": 2.669057846069336}
			],
			"166": [
				{"x": 38.51381301879883, "y": -2.072115898132324},
				{"x": 32.09283447265625, "y": -1.7142610549926758},
				{"x": 29.789670944213867, "y": 4.123003005981445},
				{"x": 40.565940856933594, "y": 3.024226188659668}
			],
			"167": [
				{"x": 43.28923034667969, "y": -2.3354134559631348},
				{"x": 36.8396110534668, "y": -2.224869728088379},
				{"x": 35.10624313354492, "y": 2.7719593048095703},
				{"x": 44.96560287475586, "y": 2.6391754150390625}
			],
			"168": [
				{"x": 16.932680130004883, "y": 2.0610780715942383},
				{"x": 6.837074279785156, "y": 1.5379981994628906},
				{"x": 6.3847126960754395, "y": 7.10891056060791},
				{"x": 16.64733123779297, "y": 8.053435325622559}
			],
			"169": [
				{"x": 1.9997296333312988, "y": 0.6303224563598633},
				{"x": -8.690858840942383, "y": 2.521585464477539},
				{"x": -8.44916820526123, "y": 7.184751033782959},
				{"x": 2.5412206649780273, "y": 7.6479997634887695}
			],
			"170": [
				{"x": 1.3306102752685547, "y": -0.5083627700805664},
				{"x": -14.243597030639648, "y": 2.7755556106567383},
				{"x": -14.391622543334961, "y": 7.404655933380127},
				{"x": 3.125298023223877, "y": 11.409119606018066}
			]
		}

		self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
		self.odd_topic = self.get_parameter("odd_topic").get_parameter_value().string_value
		self.marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value
		self.marker_ns = self.get_parameter("marker_ns").get_parameter_value().string_value
		self.line_width = self.get_parameter("line_width").get_parameter_value().double_value

		qos = QoSProfile(
			depth=10,
			reliability=ReliabilityPolicy.RELIABLE,
			durability=DurabilityPolicy.VOLATILE,
		)

		self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, qos)
		self.odd_sub = self.create_subscription(String, self.odd_topic, self._odd_cb, qos)

		self.get_logger().info(
			f"Safety node ready. Listening on '{self.odd_topic}', publishing markers on '{self.marker_topic}'"
		)

	def _odd_cb(self, msg):
		odd_cameras = self._extract_cameras_with_odds(msg.data)
		markers = self._build_polygon_markers(odd_cameras)
		self.marker_pub.publish(markers)

	def _extract_cameras_with_odds(self, raw_json):
		odd_cameras = set()

		try:
			payload = json.loads(raw_json)
		except json.JSONDecodeError:
			self.get_logger().warn("Received odd_detections is not valid JSON")
			return odd_cameras

		for item in payload:
			camera = item.get("cam")
			odd_cameras.add(str(camera))

		return odd_cameras

	def _build_polygon_markers(self, odd_cameras):
		marker_array = MarkerArray()
		stamp = self.get_clock().now().to_msg()

		offset = self.frame_position
		ox = float(offset.get("x", 0.0))
		oy = float(offset.get("y", 0.0))
		oz = float(offset.get("z", 0.0))

		for idx, (camera_name, polygon) in enumerate(self.camera_polygons.items()):
			marker = Marker()
			marker.header.frame_id = self.frame_id
			marker.header.stamp = stamp
			marker.ns = self.marker_ns
			marker.id = idx
			marker.type = Marker.LINE_STRIP
			marker.action = Marker.ADD

			marker.scale.x = self.line_width
			marker.scale.y = self.line_width
			marker.scale.z = 0.05

			# Red if this camera has odd detections, else green.
			if str(camera_name) in odd_cameras:
				marker.color.r = 1.0
				marker.color.g = 0.0
			else:
				marker.color.r = 0.0
				marker.color.g = 1.0
			marker.color.b = 0.0
			marker.color.a = 1.0

			for point in polygon:
				p = self._point(point)
				if p is None:
					continue
				# Apply offset
				p.x -= ox
				p.y -= oy
				p.z -= oz
				marker.points.append(p)

			# Close the polygon.
			if marker.points:
				marker.points.append(marker.points[0])

			marker_array.markers.append(marker)

		return marker_array

	def _point(self, xyz):
		from geometry_msgs.msg import Point

		if isinstance(xyz, dict):
			if "x" not in xyz or "y" not in xyz:
				return None
			x = xyz["x"]
			y = xyz["y"]
			z = xyz.get("z", 0.01)
		elif isinstance(xyz, (list, tuple)) and len(xyz) >= 2:
			x = xyz[0]
			y = xyz[1]
			z = xyz[2] if len(xyz) > 2 else 0.01
		else:
			return None

		p = Point()
		p.x = float(x)
		p.y = float(y)
		p.z = float(z)
		return p


def main(args=None):
	rclpy.init(args=args)
	node = SafetyNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()
