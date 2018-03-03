from aero_env import AeroEnv
import keyboard
import argparse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--serial-port-path', default='/dev/ttyACM1', help='serial port')
    args = parser.parse_args()

    env = AeroEnv(serial_port_path=args.serial_port_path)
    print('Started Aero')

    env.reset()

    while True:
        motor_speed = 100
        #         0, 1, R, G, B
        action = [0, 0, 0, 0, 0]

        # motor 0
        if keyboard.is_pressed('q'):
            print('Motor0 Up \r')
            action[0] = motor_speed
        if keyboard.is_pressed('a'):
            print('Motor0 Down \r')
            action[0] = -motor_speed

        # motor 1
        if keyboard.is_pressed('w'):
            print('Motor1 Up \r')
            action[1] = motor_speed
        elif keyboard.is_pressed('s'):
            print('Motor1 Down \r')
            action[1] = -motor_speed

        # R G B
        if keyboard.is_pressed('r'):
            print('R on \r')
            action[2] = 255
        if keyboard.is_pressed('g'):
            print('G on \r')
            action[3] = 255
        if keyboard.is_pressed('b'):
            print('B on \r')
            action[4] = 255

        state, reward, done, _ = env.step(action)
        print("STATE: Pitch: %s Yaw: %s"%(state[0], state[1]))
        print("ACTION:", action)

if __name__ == '__main__':
    main()