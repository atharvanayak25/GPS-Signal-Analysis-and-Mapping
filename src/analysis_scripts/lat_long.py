import rclpy
from rclpy.node import Node
from custommessage.msg import GPSmsg
import matplotlib.pyplot as plt

class GPS_Lat_Long(Node):
    def __init__(self):
        super().__init__('gps_lat_long')

        
        self.subscription = self.create_subscription(
            GPSmsg,
            'gps',
            self.listener_callback,
            10)
        self.subscription  

        
        self.latitude_values = []
        self.longitude_values = []

        
        plt.ion()  
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel("Longitude")
        self.ax.set_ylabel("Latitude")
        self.line, = self.ax.plot([], [], 'bo-', label="Latitude vs Longitude")
        self.ax.legend()

    def listener_callback(self, msg):
        # Append latitude and longitude values
        self.latitude_values.append(msg.latitude)
        self.longitude_values.append(msg.longitude)

        # Update the plot data
        self.line.set_xdata(self.longitude_values)
        self.line.set_ydata(self.latitude_values)
        self.ax.relim()  
        self.ax.autoscale_view()  

        plt.draw()
        plt.pause(0.01)  

def main(args=None):
    rclpy.init(args=args)

    gps_lat_long = GPS_Lat_Long()

    try:
        rclpy.spin(gps_lat_long)
    except KeyboardInterrupt:
        gps_lat_long.get_logger().info('GPS Lat Long Plotter shutting down')
    finally:
        gps_lat_long.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
