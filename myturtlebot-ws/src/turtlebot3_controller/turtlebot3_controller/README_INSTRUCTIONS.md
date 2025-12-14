# TP2 ROS2 - Navigation Autonome TurtleBot3
## instructions d'installation et d'utilisation

### contenu du package

Ce ZIP contient le code de base pour le TP2 :

```
turtlebot3_controller/
├── package.xml                      # configuration package ROS2
├── setup.py                         # configuration Python (COMPLET)
├── resource/                        # ressources ROS2
├── turtlebot3_controller/
│   ├── __init__.py
│   ├── obstacle_analyzer.py         # À COMPLÉTER (5 TODOs)
│   └── velocity_controller.py       # À COMPLÉTER (7 TODOs)
├── launch/
│   └── autonomous_navigation.launch.py  # À COMPLÉTER (2 TODOs)
└── README_INSTRUCTIONS.md           # Ce fichier
```

### installation

```bash
# 1. extraire le ZIP dans votre workspace ROS2
cd ~/ros2_tp_ws/src
unzip tp2_code_base.zip

# 2. vérifier la structure
ls turtlebot3_controller/

# 3. compiler (même avec les TODOs non complétés)
cd ~/ros2_tp_ws
colcon build --packages-select turtlebot3_controller
source install/setup.bash
```

### TODOs à Compléter

#### obstacle_analyzer.py (5 TODOs)
- **TODO 1** : créer le subscriber sur `/scan`
- **TODO 2** : créer les 3 publishers pour les zones
- **TODO 3** : créer le publisher pour l'alerte
- **TODO 4** : calculer les 3 zones (front, left, right)
- **TODO 5** : publier les distances des zones

#### velocity_controller.py (7 TODOs)
- **TODO 1** : créer les 3 subscribers pour les zones
- **TODO 2** : créer le subscriber pour l'alerte
- **TODO 3** : créer le publisher pour `/cmd_vel`
- **TODO 4-6** : implémenter la logique de contrôle
- **TODO 7** : publier la commande de vélocité

#### autonomous_navigation.launch.py (2 TODOs)
- ajouter les 2 nœuds avec leurs paramètres

### tests step by step

#### step 1 : tester obstacle_analyzer seul

```bash
# T1 : gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# T2 : votre nœud
ros2 run turtlebot3_controller obstacle_analyzer

# T3 : vérifier
ros2 topic echo /zone/front
```

**vérifications :**
- pas d'erreur au lancement
- publications sur `/zone/front`, `/zone/left`, `/zone/right`
- valeurs cohérentes (distances positives)

#### step 2 : tester velocity_controller

```bash
# T1 : gazebo (déjà actif)
# T2 : obstacle_analyzer
ros2 run turtlebot3_controller obstacle_analyzer

# T3 : velocity_controller
ros2 run turtlebot3_controller velocity_controller

# T4 : Observer
ros2 topic echo /cmd_vel
```

**vérifications :**
- Robot se déplace dans Gazebo
- Robot ralentit près des obstacles
- Robot tourne pour éviter

#### Step 3 : tester le launch file

```bash
# tout en une commande
ros2 launch turtlebot3_controller autonomous_navigation.launch.py
```

**vérifications :**
- les 2 nœuds démarrent automatiquement
- navigation autonome fonctionne

### aide au débogage

#### problème : "No module named 'turtlebot3_controller'"
```bash
# solution : recompiler et sourcer
cd ~/ros2_tp_ws
colcon build --packages-select turtlebot3_controller
source install/setup.bash
```

#### problème : "AttributeError: 'NoneType' object has no attribute 'publish'"
```bash
# cause : Vous n'avez pas créé le publisher
# vérifier les TODO 2, TODO 3, TODO 7
```

#### problème : le robot ne bouge pas
```bash
# vérifier que cmd_vel est publié
ros2 topic echo /cmd_vel

# vérifier les distances
ros2 topic echo /zone/front
```

#### problème : le robot fonce dans les murs
```bash
# cause probable : logique de contrôle TODO 4-6 incorrecte
# relire les conditions dans le sujet du TP
```

### commandes utiles

```bash
# voir tous les topics
ros2 topic list

# voir la fréquence d'un topic
ros2 topic hz /zone/front

# voir les nœuds actifs
ros2 node list

# informations sur un nœud
ros2 node info /obstacle_analyzer

# graphe visuel
rqt_graph
```

### conseils

1. **complétez les TODOs dans l'ordre** : obstacle_analyzer → velocity_controller → launch file
2. **testez après chaque TODO** : ne passez pas au suivant avant que le précédent fonctionne
3. **utilisez les logs** : `self.get_logger().info('message')` pour déboguer
4. **commencez simple** : Faites avancer le robot d'abord, puis ajoutez l'évitement

### référence API ROS2

```python
# créer un subscriber
self.sub = self.create_subscription(MessageType, 'topic_name', self.callback, 10)

# créer un publisher
self.pub = self.create_publisher(MessageType, 'topic_name', 10)

# publier un message
msg = MessageType()
msg.data = value
self.pub.publish(msg)

# créer un timer
self.timer = self.create_timer(0.1, self.callback)  # 10 Hz
```

### critères de réussite

Votre TP est réussi si :
- le robot navigue sans collision
- le robot s'arrête près des obstacles
- le robot contourne les obstacles
- le launch file démarre tout correctement
- vous répondez aux questions du sujet

Bon courage !
