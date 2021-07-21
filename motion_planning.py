import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid, prune_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    # Cuando está cerca del home en un radio de 0.1 se desarma
    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])  # Despega a la altura deseada
        # Altitud setteada en plan_path() ↓↓↓

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2],
                          self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 10  # Altitud de despegue
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv') as f:
            first_line = f.readline().strip()
        latlon = first_line.split(',')
        lon0 = float(latlon[0].strip().split(' ')[1])  # Longitud del centro
        lat0 = float(latlon[1].strip().split(' ')[1])  # Latitud del centro

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lat0, lon0, 0)
        # TODO: retrieve current global position
        global_position = self.global_position
        # TODO: convert to current local position using global_to_local()
        local_pos = global_to_local(self.global_position,
                                    global_home=self.global_home)  # Respecto al centro
        local_pos = (int(np.rint(local_pos[0])), int(np.rint(local_pos[1])), int(np.rint(local_pos[2])))
        # north, east, att = local_pos
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # TODO: convert start position to current position rather than map center
        # self.set_home_as_current_position()
        # self.set_home_position(self.global_position[0], self.global_position[1], 0)
        print("latitud global_position: ", self.global_position[0], "\tlongitud global_position: ",
              self.global_position[1])
        print("Posicion actual global_home -> ", self.global_home)
        print("Posicion local respecto al centro: ", local_pos)
        # TODO: adapt to set goal as latitude / longitude position and convert
        # Define starting point on the grid (center + vector local_pos)
        grid_start = (-north_offset + local_pos[0], -east_offset + local_pos[1])

        # Set goal as some arbitrary position on the grid
        grid_goal = self.getGoal_local_position(grid,
                                                north_offset,
                                                east_offset,
                                                local_position=local_pos)
        print('Grid Start and Goal: ', grid_start, grid_goal)

        # Run A* to find a path from start to goal
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, cost = a_star(grid, heuristic, grid_start, grid_goal)
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)

        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        pruned_path = prune_path(path)
        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset,
                      TARGET_ALTITUDE,
                      0] for p in pruned_path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def getGoal_local_position(self, grid, north_offset, east_offset, local_position):
        """
        Retorna una tupla con las coordenadas Locales de la meta

        -> return (coordenada norte, coordenada este, cordenada abajo)
        """
        # Coordenadas locales del home respecto al centro
        nh, eh, dh = local_position
        grid_shape = grid.shape  # Dimensiones del mapa (alto, ancho)
        print("Dimension del mapa -> ", grid_shape)
        # Inicializa Coordenadas de la meta
        north_coordenate, east_coordenate = None, None
        grid_goal = None
        # randomly select a goal
        dist_idx = 100.0  # Trata de garantizar que el goal generado no este fuera del mapa con una división
        goalCondition = True
        while goalCondition:  # mientras la meta sea un obstáculo se crean otras coordenadas
            # aquí estamos ignorando la generacion de la coordenada DOWN
            ng = None
            eg = None
            # GENERO la coordenada NORTE que esté dentro del mapa
            condition_north = True
            while condition_north:
                ng = self.generate_random_localPosition(north=True)
                north_coordenate = int(np.rint(ng - north_offset + nh))
                # Verifica que esté dentro del mapa
                condition_north = north_coordenate < 1 or north_coordenate > grid_shape[0] - 2
                # north_coordenate < 1 en caso que sea menor al offset
                # north_coordenate > grid_shape[0] - 2 en caso de que sea mayor al valor máximo
                # Esto considera que si está en el límite se calcula una nueva coordenada

            # GENERO la coordenada ESTE que esté dentro del mapa
            condition_east = True
            while condition_east:
                eg = self.generate_random_localPosition(east=True)
                east_coordenate = int(np.rint(eg - east_offset + eh))
                # Verifica que esté dentro del mapa
                condition_east = east_coordenate < 1 or east_coordenate > grid_shape[1] - 2
                # east_coordenate < 1 en caso que sea menor al offset
                # east_coordenate > grid_shape[1] - 2 en caso de que sea mayor al valor máximo
                # Esto considera que si está en el límite se calcula una nueva coordenada

            grid_goal = (north_coordenate, east_coordenate)  # Tupla con Coordenadas de la meta

            # Verifica si la meta calculada es un obstáculo
            obstacule_condition = grid[grid_goal[0], grid_goal[1]]  # La meta es un obstáculo?? (True->Si, False->No)
            print(grid_goal, " Es un obstáculo?-> ", obstacule_condition)

            distance_x = np.absolute(eg)
            distance_y = np.absolute(ng)
            distance = np.sqrt(distance_x ** 2 + distance_y ** 2)
            condition_distance = distance >= 100 or distance < 10  # siempre que este en ese rango, se generará otro punto
            print("\nDistancia:", distance, " , es valido?->", condition_distance)

            goalCondition = obstacule_condition or condition_distance
            print(grid_goal, "\tGenerar otro punto? -> ", goalCondition)
        print(grid_goal)
        return grid_goal

    def generate_random_localPosition(self, north=False, east=False, down=False):
        """
        Esta funcion retorna un calculo de la coordenada especificada de la meta

        Solo se debe de seleccionar una de las coordenadas asignandole el valor de True
        en el argumento
        """
        index = None
        if north:
            index = 0
            print("\nCalculating north coordenate")
        elif east:
            index = 1
            print("\nCalculating east coordenate")
        elif down:
            index = 2
            print("\nCalculating down coordenate")

        reductor = 500.0  # Trata de garantizar que el goal generado no este fuera del mapa con una división
        change = np.random.rand(1)
        change -= 0.5  # Es lo que permite que podamos ir a un norte o este positivo o negativo
        print("\tchange:", change)
        goal = (self.global_position[0] + change[0] / reductor,  # Longitud →
                self.global_position[1] + change[0] / reductor,  # Latitud  ↑
                self.global_position[2] + change[0] * 10.0)  # arriba (no se usa)

        local_goal = global_to_local(goal,
                                     global_home=self.global_position)  # array[north_coord, east_coord, down_coord]
        # recordar global_to_local retorna --> down_coord = local_goal[2]= (-1)*goal[2]
        print("\tGoal Local calculates: ", local_goal[index])
        return local_goal[index]

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=2000)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
