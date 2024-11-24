import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vincent/Downloads/course_materials/idt_ros2_nodes/src/idt/install/idt'
