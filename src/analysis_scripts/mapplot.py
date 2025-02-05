import rclpy
from rclpy.node import Node
from custommessage.msg import GPSmsg  
import folium
import os

class GPSPathPlotter(Node):
    def __init__(self):
        super().__init__('gps_path_plotter')

        self.subscription = self.create_subscription(
            GPSmsg,
            'gps',
            self.listener_callback,
            10
        )

        self.map = folium.Map(location=[0, 0], zoom_start=2)
        self.map_file = 'path_map.html'
        self.coordinates = [] 

        self.save_map()

    def listener_callback(self, msg):
        
        lat = msg.latitude  
        lon = msg.longitude 
        self.coordinates.append((lat, lon))

        folium.Marker(
            location=[lat, lon],
            popup=f'Lat: {lat}, Lng: {lon}',
            icon=folium.Icon(color='blue')
        ).add_to(self.map)

        if len(self.coordinates) > 1:
            folium.PolyLine(locations=self.coordinates, color='red', weight=2.5, opacity=0.7).add_to(self.map)

        self.save_map()

    def save_map(self):
        self.map.save(self.map_file)
        self.get_logger().info(f'Map saved to {os.path.abspath(self.map_file)}')

def main(args=None):
    rclpy.init(args=args)
    gps_path_plotter = GPSPathPlotter()

    try:
        rclpy.spin(gps_path_plotter)
    except KeyboardInterrupt:
        gps_path_plotter.get_logger().info('GPS Path Plotter shutting down')
    finally:
        gps_path_plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()