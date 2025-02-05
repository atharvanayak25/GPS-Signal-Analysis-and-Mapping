import rclpy
import utm
from std_msgs.msg import String
from std_msgs.msg import Float64
from rclpy.node import Node 
import serial
from std_msgs.msg import Header
from custommessage.msg import *
from custommessage.msg import GPSmsg

class GPS_Driver(Node):
    def __init__(self):
        super().__init__('gpsDriver')
        self.declare_parameter('port','/dev/pts/2')
        self.declare_parameter('baudrate',4800)
        self.declare_parameter('sampling_rate',10)

        serial_port = self.get_parameter('port').value
        serial_baud = self.get_parameter('baudrate').value
        serial_sampling_rate = self.get_parameter('sampling_rate').value
        
        self.port = serial.Serial(serial_port, serial_baud, timeout = 3.0)
        
        self.timer = self.create_timer( 0.1, self.read_gps_data)
        
        self.data_pub = self.create_publisher(GPSmsg, 'gps', 10)
    
    def DDMMmm_to_DD(self, DDMMmm, Direction):

        data_raw = float(DDMMmm)
        data_dd = int(data_raw/100)
        data_mm = data_raw -(data_dd * 100)

        data_converted = data_dd + (data_mm/ 60)

        if Direction == 'S' or Direction == 'W':
            data_converted = data_converted * (-1)

        return data_converted

    
    def read_gps_data(self):
        rx_data = self.port.readline().decode().strip()

        if "$GPGGA" in rx_data:
            
            components = rx_data.split(',')

            msg = GPSmsg()


            utc = components[1]
            
            lat = float(components[2])
            lat_dir = str(components[3])
            

            
            long = float(components[4])
            long_dir =str(components[5])
            


            altitude = float(components[9])

            hh = int(utc[:2])
            mm = int(utc[2:4])
            secs = float(utc[4:])

            utc_seconds = int(hh * 3600 + mm * 60 + (secs))
            utc_nanoseconds = int((secs - int(secs)) * 1e9)

            lat_converted = self.DDMMmm_to_DD(lat, lat_dir)
            long_converted = self.DDMMmm_to_DD(long, long_dir)

            utm_latlong = utm.from_latlon(lat_converted, long_converted)

            msg.header.stamp.sec = utc_seconds
            msg.header.stamp.nanosec = utc_nanoseconds
            msg.header.frame_id = 'GPS1_Frame'
            msg.latitude = lat_converted
            msg.longitude = long_converted
            msg.altitude = altitude
            msg.utm_easting = utm_latlong[0]
            msg.utm_northing = utm_latlong[1]
            msg.zone = utm_latlong[2]
            msg.letter = utm_latlong[3]

            
            
            self.data_pub.publish(msg)


            self.get_logger().info(f"Current GPS Status: {msg}")

def main(args=None):
    rclpy.init(args=args)
    gps_driver = GPS_Driver()

    try:
        rclpy.spin(gps_driver)
    except KeyboardInterrupt:
        gps_driver.get_logger().info('GPS Driver shutting down')
    finally:
        gps_driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()









