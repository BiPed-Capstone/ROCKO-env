import pygame
import struct
import socket
import time
 
# Set up socket
ROCKO_IP = "137.112.206.241"
ROCKO_PORT = 1234
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
 
sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
 
# sock.connect((ROCKO_IP, ROCKO_PORT))
 
# Set up joystick
pygame.init()
pygame.joystick.init()
 
num_joysticks = pygame.joystick.get_count()
 
if num_joysticks > 0:
    controller = pygame.joystick.Joystick(0)
    controller.init()
    print("Controller connected:", controller.get_name())
else:
    print("No controller detected.")
    exit(1)
 
# To get the number of axes, buttons, and hats on the controller
num_axes = controller.get_numaxes()
num_buttons = controller.get_numbuttons()
num_hats = controller.get_numhats()
 
print(num_axes)
print(num_buttons)
 
def get_data():
    pygame.event.pump()
    axes = [controller.get_axis(i) for i in range(num_axes)]
    buttons = [controller.get_button(i) for i in range(num_buttons)]
 
    # pack = struct.pack(f"{len(axes)}f {len(buttons)}B", *axes, *buttons)
   
    # print(axes)
    # print(buttons)
    # return pack
    return axes, buttons
 
 
while True:
    try:
        sock.connect((ROCKO_IP, ROCKO_PORT))
        print("Connected to Pi")
        break
    except socket.error:
            print("Couldnt connect with the socket, retrying")
            time.sleep(1)
 
prev_axes = [0.0] * num_axes
prev_buttons = [0] * num_buttons
packet = struct.pack(f"{len(prev_axes)}f {len(prev_buttons)}B", *prev_axes, *prev_buttons)
sock.sendall(packet)
 
while True:
    # packet = get_data()
    axes, buttons = get_data()
    # sock.sendall(packet)
 
    if axes != prev_axes or buttons != prev_buttons:
        prev_axes = axes[:]
        prev_buttons = buttons[:]
 
        packet = struct.pack(f"{len(axes)}f {len(buttons)}B", *axes, *buttons)
        sock.sendall(packet)
 
#     for event in pygame.event.get():
#         axis_0 = controller.get_axis(0)
#         axis_1 = -controller.get_axis(1)
#         axis_2 = controller.get_axis(2)
#         axis_3 = -controller.get_axis(3)
 
#         button_0 = controller.get_button(0)
#         button_1 = controller.get_button(1)
 
#         print("Left X", axis_0)
#         print("Left Y", axis_1)
#         print("Right X", axis_2)
#         print("Right Y", axis_3)
#         print("Bottom Button", button_0)
#         print("Left Button", button_1)
   
# pygame.quit()