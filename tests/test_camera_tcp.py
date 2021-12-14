from real.camera import Camera
import numpy as np

cam = Camera(port=50000)

# Ping the server with anything
cam.tcp_socket.send(b'asdf')

data = b''
while len(data) < (10 * 4 + cam.im_height * cam.im_width * 5):
    print("Waiting for data...")
    data += cam.tcp_socket.recv(cam.buffer_size)
    print("Received data...")

dummy_serial = np.fromstring(data[0:(6*4)], np.int32)

print(dummy_serial)