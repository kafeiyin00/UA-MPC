import rospy
from std_msgs.msg import Float32
from message_filters import ApproximateTimeSynchronizer, Subscriber

class TimePerformanceLogger:
    def __init__(self):
        rospy.init_node('time_performance_logger', anonymous=True)
        self.file = open('/workspace/ws_rotor_sim/src/RotorOptimalControl/system_evaluation/log/timeperformance.txt', 'a')
        
        self.control_sub = Subscriber('/control_elapsed_time', Float32)
        self.rendering_sub = Subscriber('/rendering_elapsed_time', Float32)
        self.processing_sub = Subscriber('/processing_time_lo', Float32)
        
        self.ts = ApproximateTimeSynchronizer(
            [self.control_sub, self.rendering_sub, self.processing_sub], 
            queue_size=10, 
            slop=0.1
        )
        self.ts.allow_headerless = True
        self.ts.registerCallback(self.log_times)
        
    def log_times(self, control_msg, rendering_msg, processing_msg):
        timestamp = rospy.get_time()
        self.file.write(f"{timestamp}, {control_msg.data}, {rendering_msg.data}, {processing_msg.data}\n")
        
    def run(self):
        rospy.spin()
        self.file.close()

if __name__ == '__main__':
    logger = TimePerformanceLogger()
    logger.run()
