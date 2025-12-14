#!/usr/bin/env python3
"""
Velocity Controller Node for TurtleBot3
Controls robot movement based on obstacle detection
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist


class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        
        # TODO 1: Créer les subscribers pour les 3 zones
        self.left_sub = self.create_subscription(Float32, '/zone/left', self.left_callback, 10)
        self.front_sub = self.create_subscription(Float32, '/zone/front', self.front_callback, 10)
        self.right_sub = self.create_subscription(Float32, '/zone/right', self.right_callback, 10)
        
        # TODO 2: Créer le subscriber pour l'alerte
        self.alert_sub = self.create_subscription(Float32, '/obstacle_alert', self.alert_callback, 10)

        # TODO 3: Créer le publisher pour /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Paramètres de vitesse
        self.declare_parameter('max_linear_speed', 0.2)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('comfort_distance', 0.8)
        
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.safe_dist = self.get_parameter('safe_distance').value
        self.comfort_dist = self.get_parameter('comfort_distance').value
        
        # Variables d'état (distances des obstacles)
        self.left_distance = float('inf')
        self.front_distance = float('inf')
        self.right_distance = float('inf')
        self.obstacle_detected = False
        
        # Timer de contrôle à 10 Hz (0.1 secondes)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(
            f'Velocity Controller démarré\n'
            f'  Vitesse max: {self.max_linear} m/s\n'
            f'  Distance sûre: {self.safe_dist} m\n'
            f'  Distance confort: {self.comfort_dist} m'
        )
    
    def left_callback(self, msg):
        """Callback pour la zone gauche"""
        self.left_distance = msg.data
    
    def front_callback(self, msg):
        """Callback pour la zone frontale"""
        self.front_distance = msg.data
    
    def right_callback(self, msg):
        """Callback pour la zone droite"""
        self.right_distance = msg.data
    
    def alert_callback(self, msg):
        """Callback pour l'alerte obstacle"""
        self.obstacle_detected = msg.data
    
    def control_loop(self):
        """
        Boucle de contrôle principale - appelée toutes les 0.1 secondes
        Implémente la logique d'évitement d'obstacles
        """
        # Créer le message de commande
        cmd = Twist()
        
        # TODO 4: Implémenter la logique de contrôle
        # Règle 1: Si front < safe_dist (0.5m) → Stop et tourner
        # Règle 2: Si front < comfort_dist (0.8m) → Ralentir
        # Règle 3: Sinon → Avancer à vitesse max
        
        # TODO 5: Décider la direction de rotation
        # Si left > right → tourner à gauche (angular.z positif)
        # Si right > left → tourner à droite (angular.z négatif)
        
        
        if self.front_distance < self.safe_dist:
            # Danger immédiat - Stop et tourner
            cmd.linear.x = 0.0
            if self.left_distance > self.right_distance:
                cmd.angular.z = self.max_angular  # Tourner gauche
            else:
                cmd.angular.z = -self.max_angular  # Tourner droite
        elif self.front_distance < self.comfort_dist:
            # Approche obstacle - Ralentir
            cmd.linear.x = self.max_linear * 0.5
            # Peut-être commencer à tourner légèrement
        else:
            # Voie libre - Avancer
            cmd.linear.x = self.max_linear
            cmd.angular.z = 0.0
        
        
        # TODO 7: Publier la commande

        # Utilisez: self.cmd_vel_pub.publish(cmd)
        self.cmd_vel_pub.publish(cmd)
        #self.get_logger().info('Publishing: "%s"' % cmd)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Arrêter le robot proprement
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
