import cv2
import numpy as np
import socket
import threading
import queue
import time
import traceback
import pygame
import sys

CMD_UDP_PORT = 3197
CMD_LOOP_TIME = 0.1
IN_MAX = -1.0
IN_MIN = 1.0
OUT_MAX = 255.0
OUT_MIN = 0.0

frame_q = queue.Queue()
running = True


def scale_in_range(value: float):
    return round(((OUT_MAX - OUT_MIN) * ((value - IN_MIN) / (IN_MAX - IN_MIN))) + OUT_MIN)


def get_commands(controller: pygame.joystick):
    axes = controller.get_numaxes()
    buttons = controller.get_numbuttons()
    hats = controller.get_numhats()
    output = [0] * (axes + buttons + hats * 2)

    for num in range(axes):
        output[num] = scale_in_range(controller.get_axis(num))
    for num in range(buttons):
        output[axes + num] = round(controller.get_button(num) * OUT_MAX)
    for num in range(hats):
        output[axes + buttons + num * 2] = scale_in_range(controller.get_hat(num)[0])
        output[axes + buttons + num * 2 + 1] = scale_in_range(controller.get_hat(num)[1])

    return output


def init() -> pygame.joystick:
    pygame.init()
    joystick_count = pygame.joystick.get_count()
    print("There is " + str(joystick_count) + " joystick/s")

    if joystick_count == 0:
        # if no joysticks, quit program safely
        print("Error, I did not find any joysticks")
        pygame.quit()
        sys.exit()
    else:
        # initialise joystick
        controller = pygame.joystick.Joystick(0)
        controller.init()

    print("There is " + str(controller.get_numaxes()) + " axes")
    print("There is " + str(controller.get_numbuttons()) + " button/s")
    print("There is " + str(controller.get_numhats()) + " hat/s")

    return controller


def parse_commands_car(c: list):
    # [R2, L2, ARROW_HOR]!
    # Always use 3 digits per number
    parsed_mess = "[" + "{:03d}".format(c[5]) + \
                  ";" + "{:03d}".format(c[4]) + \
                  ";" + "{:03d}".format(c[0]) + "]|"
    return parsed_mess


def udp_recv(listen_addr, target_addr):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1)
    sock.bind((listen_addr, 55556))

    print('Start Streaming...')

    chunks = b''
    while running:
        try:
            msg, address = sock.recvfrom(2 ** 16)
        except Exception as e:
            sock.sendto(b'\x55', (target_addr, 55555))
            continue
        soi = msg.find(b'\xff\xd8\xff')
        eoi = msg.rfind(b'\xff\xd9')
        if soi >= 0:
            if chunks.startswith(b'\xff\xd8\xff'):
                if eoi >= 0:
                    chunks += msg[:eoi + 2]
                    eoi = -1
                else:
                    chunks += msg[:soi]
                try:
                    frame_q.put(chunks, timeout=1)
                except Exception as e:
                    print(e)
            chunks = msg[soi:]
        else:
            chunks += msg
        if eoi >= 0:
            eob = len(chunks) - len(msg) + eoi + 2
            if chunks.startswith(b'\xff\xd8\xff'):
                byte_frame = chunks[:eob]
                try:
                    frame_q.put(byte_frame, timeout=1)
                except Exception as e:
                    print(e)
            else:
                print('[', time.perf_counter(), "] Invalid picture")
            chunks = chunks[eob:]
    sock.close()
    print('Stop Streaming')


def main(args):
    global running

    # Init components
    joystick = init()
    listener = socket.gethostbyname(socket.gethostname())
    sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("Sending commands to " + listener + ":" + str(CMD_UDP_PORT))

    thread = threading.Thread(target=udp_recv, args=(listener, args.target))
    thread.start()

    win_name = "FPV CAMERA - Press 'q' to quit"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    if args.fullscreen:
        cv2.setWindowProperty(win_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    writer = None
    img = None
    next_write = 0
    last_time = time.time()
    while True:

        # Joystick command
        if (time.time() - last_time > CMD_LOOP_TIME):
            pygame.event.get()
            commands = get_commands(joystick)
            comm_mess = parse_commands_car(commands)

            sock_cmd.sendto(comm_mess.encode("utf-8"), (args.target, CMD_UDP_PORT))
            last_time = time.time()

        try:
            if args.write:
                if not frame_q.empty():
                    img = None
            else:
                img = None
            if img is None:
                while True:
                    byte_frame = frame_q.get(block=True, timeout=1)
                    if frame_q.empty() or args.grab_all:
                        break
                    print('[', time.perf_counter(), '] Skip picture')
                img = cv2.imdecode(np.frombuffer(byte_frame, dtype=np.uint8), 1)

                # rotate
                # img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
                # img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

                # resize
                # width = 800
                # h, w = img.shape[:2]
                # if w < width:
                #     print(time.perf_counter(), 'Resize picture')
                #     height = round(h * (width / w))
                #     img = cv2.resize(img, dsize=(width, height))

            if args.write:
                if writer is None:
                    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
                    writer = cv2.VideoWriter(args.write, fourcc, args.fps, (img.shape[1], img.shape[0]))
                if time.perf_counter() > next_write:
                    next_write += 1 / args.fps
                    writer.write(img)

            cv2.imshow(win_name, img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        except queue.Empty as e:
            pass
        except Exception as e:
            print(traceback.format_exc())
        except KeyboardInterrupt as e:
            print('KeyboardInterrupt')
            break

    if writer:
        writer.release()
    print('Waiting for recv thread to end')
    running = False
    thread.join()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("target", type=str)
    parser.add_argument("--fullscreen", action='store_true')
    parser.add_argument("--write", type=str)
    parser.add_argument("--fps", type=int, default=60)
    parser.add_argument("--grab-all", action='store_true', default=False)
    args = parser.parse_args()
    main(args)
