#!/usr/bin/env python3
"""
Obstacle Analyzer Node for TurtleBot3
Analyzes laser scan data and detects obstacles in 3 zones
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool
import math


class ObstacleAnalyzer(Node):
    def __init__(self):
        super().__init__('obstacle_analyzer')
        
        # TODO 1: Créer un subscriber sur le topic /scan
        # Utilisez: self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # TODO 2: Créer les publishers pour les 3 zones
        # Utilisez: self.create_publisher(Float32, '/zone/front', 10)
        self.front_pub = self.create_publisher(Float32, '/zone/front', 10)
        self.left_pub = self.create_publisher(Float32, '/zone/left', 10)
        self.right_pub = self.create_publisher(Float32, '/zone/right', 10)
        
        # TODO 3: Créer le publisher pour l'alerte obstacle
        self.alert_pub = self.create_publisher(Bool, '/obstacle_alert', 10)
        
        # Paramètre de seuil de danger
        self.declare_parameter('danger_threshold', 0.5)
        self.danger_threshold = self.get_parameter('danger_threshold').value
        
        self.get_logger().info(f'Obstacle Analyzer démarré - Seuil: {self.danger_threshold}m')
    
    def scan_callback(self, msg):
        """Callback appelé à chaque nouveau scan laser"""
        ranges = msg.ranges
        
        # Vérifier que nous avons 360 mesures
        if len(ranges) < 360:
            self.get_logger().warn('Données laser invalides')
            return
        
        # TODO 4: Calculer les 3 zones
        # Zone FRONT: indices 315-360 et 0-45
        # Astuce: front_ranges = list(ranges[315:360]) + list(ranges[0:45])
        front_ranges = list(ranges[315:360]) + list(ranges[0:45])
        
        # Zone LEFT: indices 45-135
        left_ranges = list(ranges[45:135])

        # Zone RIGHT: indices 225-315
        right_ranges = list(ranges[225:315])
        
        # Calculer les distances minimales (filtrer les valeurs inf/nan)
        front_min = self.get_min_distance(front_ranges, msg.range_max)
        left_min = self.get_min_distance(left_ranges, msg.range_max)
        right_min = self.get_min_distance(right_ranges, msg.range_max)
        
        # TODO 5: Publier les distances des zones
        # Créer des messages Float32 et publier
        # Exemple: front_msg = Float32(); front_msg.data = front_min; self.front_pub.publish(front_msg)

        front_msg = Float32()
        front_msg.data = front_min
        self.front_pub.publish(front_msg)

        left_msg = Float32() 
        left_msg.data = left_min
        self.left_pub.publish(left_msg)

        right_msg = Float32()
        right_msg.data = right_min
        self.right_pub.publish(right_msg)
        
        # Vérifier si obstacle détecté
        obstacle_detected = (front_min < self.danger_threshold or 
                            left_min < self.danger_threshold or 
                            right_min < self.danger_threshold)
        
        # Publier l'alerte
        alert_msg = Bool()
        alert_msg.data = obstacle_detected
        self.alert_pub.publish(alert_msg)
        
        # Log si obstacle détecté
        if obstacle_detected:
            self.get_logger().warn(
                f'OBSTACLE! Front: {front_min:.2f}m, Left: {left_min:.2f}m, Right: {right_min:.2f}m',
                throttle_duration_sec=1.0
            )
    
    def get_min_distance(self, ranges, max_range):
        """
        Trouve la distance minimale dans une liste de mesures
        Filtre les valeurs inf et nan
        """
        valid_ranges = [r for r in ranges 
                       if r < max_range and not math.isnan(r) and not math.isinf(r)]
        
        if len(valid_ranges) == 0:
            return max_range
        
        return min(valid_ranges)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAnalyzer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
