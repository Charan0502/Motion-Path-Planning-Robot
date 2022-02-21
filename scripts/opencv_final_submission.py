from simple_pid import PID
import numpy as np
import cv2
import time
import paho.mqtt.client as mqtt

global flag
flag = 0

vid = cv2.VideoCapture(1)

kernel = np.ones((9, 9), np.uint8)

width, height = 1920, 1080

pts1 = np.float32([[392, 388 - 40], [1320, 456 - 40], [5, 970], [1547, 997-30]])
pts2 = np.float32([[-25, 0], [width + 10, 0], [0, height], [width + 10, height]])
broker_address = "192.168.1.25"

client = mqtt.Client("MAC")
client.connect(broker_address)

client.subscribe("botpinkf", qos=0)


def on_message(client, userdata, msg):
    # if(str(msg.payload)=='T'):
    print(str(msg.payload.decode("utf-8")))
    flag = 1


days = 0
hrs = 0
mins = 0
sec = 0
period = '00:00:00:00'

pid = PID(Kp=0, Kd=0, Ki=0, setpoint=0, sample_time=0.01)


class Bots:
    def __init__(self, name, lower, upper, center, turncoord, releasecoord, turncoord2, stopcoord, straightcorrection, Turncorrection):
        self.name = name
        self.lower = lower
        self.upper = upper
        self.center = center
        self.turncoord = turncoord
        self.releasecoord = releasecoord
        self.turncoord2 = turncoord2
        self.stopcoord = stopcoord
        self.straightcorrection = straightcorrection
        self.Turncorrection = Turncorrection

    def DefineCenter(self, hsv, img):
        mask = cv2.inRange(hsv, self.lower, self.upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            # ((self.center[0], self.center[1]), radius) = cv2.minEnclosingCircle(c)
            x, y, w, h = cv2.boundingRect(c)
            self.center[0] = x + w / 2
            self.center[1] = y + h / 2
            if w > 5 and h > 5:
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.circle(img, (int(x + w / 2), int(y + h / 2)), 0, (255, 0, 0), 15)
                cv2.putText(img, self.name, (int(self.center[0]), int(self.center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 0, 0), 2)
        return self.center

colors = [

    Bots("botyellow",(23, 59, 119), (54, 255, 255), [0, 0], (790, 277+50), (77+0, 285 + 50), (1221, 1301), (785, 1003),30, -130),
    Bots("botpink", (124, 50, 70), (179, 255, 255), [0, 0], (869, 173+100), (79+0, 201), (width, height), (875, 1003), 30, -50),
    Bots("botpink", (124, 50, 70), (179, 255, 255), [0, 0], (969, 259), (1793, 175 + 50), (600, 660), (970, 980), 30, -50),
    Bots("botOrange",(10, 50, 70), (24, 255, 255), [0, 0], (1062, 155), (1795, 265 + 50), (1221, 1301), (1065, 1003), 0, -150),




    # Bots("botyellow", (23, 59, 119), (54, 255, 255), [0, 0], (827, 270), (106, 265 + 50), (1221, 1301), (825, 980), 20,-100, ),
    # Bots("botblue", (23, 59, 119), (54, 255, 255), [0, 0], (250, 320), (100, 180), (748, 803), (840, 901), 0, 0)
    # Bots("botpink", (124, 50, 70), (179, 255, 255), [0, 0], (917, 270), (106, 265 + 50), (600, 660), (850, 890), 0, 0),
    # Bots("botorange", (23, 59, 119), (54, 255, 255), [0, 0], (920, 270), (110, 183 + 50), (1221, 1301), (915, 1000), 30,0),

    # (10, 50, 70), (24, 255, 255) orange
    # (23, 59, 119), (54, 255, 255) yellow
    # (124, 50, 70), (179, 255, 255) pink
    # (65, 150, 0), (120, 255, 255) blue
]

num_of_frames = 1


def Pmeasure(a, b, cp, ci, cd):
    pid.setpoint = b
    pid.tunings = (cp, ci, cd)
    steer = pid(a)
    return steer


def main():
    steps = 0
    Clock = 0
    i = 1
    j = 0
    a = 0

    while True:
        suc, img = vid.read()
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        img = cv2.warpPerspective(img, matrix, (width, height))
        # img = cv2.resize(img, (800, 800))
        blur = cv2.GaussianBlur(img, (11, 11), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        center = colors[j].DefineCenter(hsv, img)
        # client.loop_forever()
        client.on_message = on_message
        # client.loop_stop()
        # img = cv2.circle(img, (center[0], center[1]), radius=0, color=(125, 125, 125), thickness=-1)
        cv2.line(img, (colors[j].turncoord[0], 0), (colors[j].turncoord[0], height), (0, 255, 255), 1)
        cv2.line(img, (0, colors[j].turncoord[1] - 50), (width, colors[j].turncoord[1] - 50), (0, 255, 255), 1)
        cv2.line(img, (0, colors[j].turncoord[1] + 800), (width, colors[j].turncoord[1] + 800), (0, 255, 255), 1)
        cv2.line(img, (colors[j].releasecoord[0] + 50, 0), (colors[j].releasecoord[0] + 50, height), (0, 255, 255), 1)
        cv2.line(img, (colors[j].turncoord[0] - 100, 0), (colors[j].turncoord[0] - 100, height), (0, 255, 255), 1)

        if a == 0:
            client.publish(colors[j].name, '4');
            a = 1

        # 1st Turning
        if j < 2:
            if colors[j].turncoord[1] + 820 > center[1] > colors[j].turncoord[1] and steps == 0:

                measure = Pmeasure(center[0], colors[j].turncoord[0] + colors[j].straightcorrection, 1.7, 0, 4)
                if j==0:
                    Rpwm = 300 + measure
                    Lpwm = 300 - measure
                if j==1:
                    Rpwm = 300 - measure
                    Lpwm = 300 + measure
                if Rpwm < 100:
                    Rpwm = 101
                if Lpwm < 100:
                    Lpwm = 101
                client.publish(colors[j].name, str(round(Rpwm)))  # Right Motor n
                client.publish(colors[j].name, str(round(Lpwm)))  # Left Motor
                print(center[0], colors[j].turncoord[0])
                print(pid.Kp)
                print(pid.Kd)

            elif center[1] < colors[j].turncoord[1] and steps == 0:
                client.publish(colors[j].name, "100")
                print(100)  # PID control goes to MPU
                if j < 2:
                    client.publish(colors[j].name, "2")  # MPU Left turn
                elif j > 2:
                    client.publish(colors[j].name, "1")  # MPU Left turn

                print("mf")
                steps += 1

            elif colors[j].releasecoord[0] + 50 < center[0] < colors[j].turncoord[0] - 200 and steps == 1:
                pid.Kp = 0
                # pid.Ki = 2
                pid.Kd = 0
                measure = Pmeasure(center[1], colors[j].releasecoord[1] + colors[j].Turncorrection, 1.7, 0, 4)
                print(center[1], colors[j].turncoord[1] - 40)
                print(pid.Kp)
                print(pid.Ki)
                print(pid.Kd)
                if j == 0 :
                    Rpwm = 300 - measure
                    Lpwm = 300 + measure
                if j == 1 :
                    Rpwm = 300 - measure
                    Lpwm = 300 + measure
                if Rpwm < 100:
                    Rpwm = 101
                if Lpwm < 100:
                    Lpwm = 101
                client.publish(colors[j].name, str(round(Rpwm)))  # Right Motor
                client.publish(colors[j].name, str(round(Lpwm)))  # Left Motor
                print(measure)

            elif center[0] < colors[j-1].releasecoord[0] + 150 and steps == 1:
                client.publish(colors[j].name, "100")
                print(100)  # PID control goes to MPU
                client.publish(colors[j].name, "5")  # MPU U turn
                steps += 1

            elif colors[j].releasecoord[0] + 100 < center[0] < colors[j].turncoord[0] - 150 and steps == 2:
                measure = Pmeasure(center[1], colors[j].releasecoord[1] + colors[j].Turncorrection, 1.7, 0, 4)

                if j == 0:
                    Rpwm = 300 + measure
                    Lpwm = 300 - measure
                if j == 1:
                    Rpwm = 300 - measure
                    Lpwm = 300 + measure
                if Rpwm < 100:
                    Rpwm = 101
                if Lpwm < 100:
                    Lpwm = 101
                client.publish(colors[j].name, str(round(Rpwm)))  # Right Motor
                client.publish(colors[j].name, str(round(Lpwm)))  # Left Motor
                print(measure)

            elif center[0] > colors[j].turncoord[0] - 50 and steps == 2:
                client.publish(colors[j].name, "100")
                print(100)  # PID control goes to MPU
                if j < 2:
                    client.publish(colors[j].name, "1")  # MPU U turn
                elif j > 2:
                    client.publish(colors[j].name, "2")
                steps += 1

            elif colors[j].turncoord[1] + 200 < center[1] < colors[j].stopcoord[1] - 50 and steps == 3:
                # colors[j].turncoord[1] + 820 > center[1] > colors[j].turncoord[1] + 30
                # colors[j].turncoord[1] + 200 < center[1] < colors[j].stopcoord[1] - 50
                # center[0], colors[j].turncoord[0] + colors[j].straightcorrection,
                # measure = Pmeasure(center[0], colors[j].turncoord[0] + colors[j].straightcorrection, 1.7, 0, 4)

                measure = Pmeasure(center[0], colors[j].turncoord[0] + colors[j].straightcorrection, 1.7, 0, 4)
                print("punagai")
                print(center[0], colors[j].releasecoord[0])
                if j == 0:
                    Rpwm = 300 - measure
                    Lpwm = 300 + measure
                if j == 1:
                    Rpwm = 300 + measure
                    Lpwm = 300 - measure
                if Rpwm < 100:
                    Rpwm = 101
                if Lpwm < 100:
                    Lpwm = 101
                client.publish(colors[j].name, str(round(Rpwm)))  # Right Motor
                client.publish(colors[j].name, str(round(Lpwm)))  # Left Motor
                print(measure)

            elif center[1] > colors[j].stopcoord[1] - 50 and steps == 4:
                client.publish(colors[j].name, "100")
                client.publish(colors[j].name, "8")
                client.publish(colors[j].name, "0")
                j += 1
                steps = 0
        else:
            if colors[j].turncoord[1] + 820 > center[1] > colors[j].turncoord[1] and steps == 0:

                measure = Pmeasure(center[0], colors[j].turncoord[0] + colors[j].straightcorrection, 1.7, 0, 4)
                Rpwm = 300 - measure
                Lpwm = 300 + measure
                if Rpwm < 100:
                    Rpwm = 101
                if Lpwm < 100:
                    Lpwm = 101
                client.publish(colors[j].name, str(round(Rpwm)))  # Right Motor n
                client.publish(colors[j].name, str(round(Lpwm)))  # Left Motor
                print(center[0], colors[j].turncoord[0])
                print(pid.Kp)
                print(pid.Kd)

            elif center[1] < colors[j].turncoord[1] and steps == 0:
                client.publish(colors[j].name, "100")
                print(100)  # PID control goes to MPU
                if j < 2:
                    client.publish(colors[j].name, "2")  # MPU Left turn
                elif j > 2:
                    client.publish(colors[j].name, "1")  # MPU Left turn

                print("mf")
                steps += 1

            elif colors[j].releasecoord[0] - 50 > center[0] > colors[j].turncoord[0] + 200 and steps == 1:
                pid.Kp = 0
                # pid.Ki = 2
                pid.Kd = 0
                measure = Pmeasure(center[1], colors[j].releasecoord[1] + colors[j].Turncorrection, 1.7, 0, 4)
                print(center[1], colors[j].turncoord[1] - 40)
                print(pid.Kp)
                print(pid.Ki)
                print(pid.Kd)
                Rpwm = 300 - measure
                Lpwm = 300 + measure
                if Rpwm < 100:
                    Rpwm = 101
                if Lpwm < 100:
                    Lpwm = 101
                client.publish(colors[j].name, str(round(Rpwm)))  # Right Motor
                client.publish(colors[j].name, str(round(Lpwm)))  # Left Motor
                print(measure)

            elif center[0] > colors[j].releasecoord[0] - 50 and steps == 1:
                client.publish(colors[j].name, "100")
                print(100)  # PID control goes to MPU
                client.publish(colors[j].name, "5")  # MPU U turn
                steps += 1

            elif colors[j].releasecoord[0] - 100 > center[0] > colors[j].turncoord[0] + 150 and steps == 2:
                measure = Pmeasure(center[1], colors[j].releasecoord[1] - colors[j].Turncorrection, 1.7, 0, 4)
                Rpwm = 300 + measure
                Lpwm = 300 - measure
                if Rpwm < 100:
                    Rpwm = 101
                if Lpwm < 100:
                    Lpwm = 101
                client.publish(colors[j].name, str(round(Rpwm)))  # Right Motor
                client.publish(colors[j].name, str(round(Lpwm)))  # Left Motor
            #     print(measure)

            elif center[0] < colors[j].turncoord[0] + 50 and steps == 2:
                client.publish(colors[j].name, "100")
                print(100)  # PID control goes to MPU
                if j < 2:
                    client.publish(colors[j].name, "1")  # MPU U turn
                elif j > 2:
                    client.publish(colors[j].name, "2")
                steps += 1

            elif colors[j].turncoord[1] + 200 > center[1] > colors[j].stopcoord[1] + 30 and steps == 3:
                colors[j].turncoord[1] + 820 > center[1] > colors[j].turncoord[1] + 30
                colors[j].turncoord[1] + 200 < center[1] < colors[j].stopcoord[1] - 50
                measure = Pmeasure(center[0], colors[j].releasecoord[0], 1.7, 0, 4)
                measure = Pmeasure(center[0], colors[j].turncoord[0] + colors[j].straightcorrection, 1.7, 0, 4)
                print("punagai2")
                print(center[0], colors[j].releasecoord[0])
                Rpwm = 300 - measure
                Lpwm = 300 + measure
                if Rpwm < 100:
                    Rpwm = 101
                if Lpwm < 100:
                    Lpwm = 101
                client.publish(colors[j].name, str(round(Rpwm)))  # Right Motor
                client.publish(colors[j].name, str(round(Lpwm)))  # Left Motor
                print(measure)

            elif center[1] > colors[j].stopcoord[1] - 50 and steps == 4:
                client.publish(colors[j].name, "100")
                client.publish(colors[j].name, "8")
                client.publish(colors[j].name, "0")
                j += 1
                steps = 0

        cv2.imshow("real", img)
        if cv2.waitKey(1) and 0xFF == ord('q'):
            break
    vid.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
