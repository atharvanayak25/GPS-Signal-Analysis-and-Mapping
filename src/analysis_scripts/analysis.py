import rclpy
from rclpy.node import Node
from custommessage.msg import GPSmsg
import matplotlib.pyplot as plt

class GPS_Plotter(Node):
    def __init__(self):
        super().__init__('gps_plotter')
        
        
        self.subscription = self.create_subscription(
            GPSmsg,
            'gps',
            self.listener_callback,
            10)
        
        self.subscription  
        
        
        self.easting_values = []
        self.northing_values = []
        
        
        plt.ion()  
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel("UTM_Easting")
        self.ax.set_ylabel("UTM_Northing")
        self.line, = self.ax.plot([], [], 'bo-', label="GPS Co-ordinates")
        self.ax.legend()
    
    def listener_callback(self, msg):
        
        self.easting_values.append(msg.utm_easting)
        self.northing_values.append(msg.utm_northing)
        
        
        self.line.set_xdata(self.easting_values)
        self.line.set_ydata(self.northing_values)
        self.ax.relim()  
        self.ax.autoscale_view()  
        plt.draw()
        plt.pause(0.01)  


def main(args=None):
    rclpy.init(args=args)
    
    
    gps_plotter = GPS_Plotter()

    try:
        rclpy.spin(gps_plotter)
    except KeyboardInterrupt:
        gps_plotter.get_logger().info('GPS Plotter shutting down')
    finally:
        gps_plotter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
