import struct
import socket
import time

# Set up socket
ROCKO_IP = "137.112.206.241"
ROCKO_PORT = 1234

# HEARTBEAT (finding C-3). The robot's DiffDriveController zeroes the velocity reference when it has
# not received a gamepad command for command_timeout_s (0.2 s), so that a dropped link stops the
# robot instead of leaving it driving on the last command. This script used to send only when the
# stick or a button CHANGED, so a stick held steady would look exactly like a dropped link and stop
# the robot 0.2 s later. So the same packet is also resent every HEARTBEAT_PERIOD_S while nothing
# changes: 0.05 s is 4 heartbeats inside one timeout, so a single late packet does not stop the
# robot. The packet itself (axes as float32, then buttons as uint8; 40 bytes for 6 axes and 16
# buttons) is unchanged, so Joystick.py on the Pi needs no change.
HEARTBEAT_PERIOD_S = 0.05

# The robot's Joystick.py reads EXACTLY this many axes and buttons per message (40 bytes). A gamepad
# with fewer is padded with 0, one with more is cut short, so every message is the same size and the
# robot never loses its place in the byte stream.
NUM_AXES = 6
NUM_BUTTONS = 16


def pack(axes, buttons):
    axes = (list(axes) + [0.0] * NUM_AXES)[:NUM_AXES]
    buttons = (list(buttons) + [0] * NUM_BUTTONS)[:NUM_BUTTONS]
    return struct.pack(f"{NUM_AXES}f {NUM_BUTTONS}B", *axes, *buttons)


def send_loop(sock, first_packet, read_packet, now=time.monotonic, keep_going=lambda: True):
    """Send first_packet, then read_packet()'s result whenever it differs from the last one sent,
    and the last one again whenever HEARTBEAT_PERIOD_S has passed without a send.
    now and keep_going are parameters only so a test can run this without a gamepad."""
    packet = first_packet
    sock.sendall(packet)
    last_sent_s = now()
    while keep_going():
        new_packet = read_packet()
        if new_packet != packet or now() - last_sent_s >= HEARTBEAT_PERIOD_S:
            packet = new_packet
            sock.sendall(packet)
            last_sent_s = now()


if __name__ == "__main__":
    # Imported here, not at the top, so a test can import send_loop without pygame installed.
    import pygame

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
        return axes, buttons

    while True:
        try:
            sock.connect((ROCKO_IP, ROCKO_PORT))
            print("Connected to Pi")
            break
        except socket.error:
                print("Couldnt connect with the socket, retrying")
                time.sleep(1)

    # Start from "nothing pressed", as before, then send the gamepad's state.
    send_loop(sock, pack([0.0] * num_axes, [0] * num_buttons), lambda: pack(*get_data()))

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
