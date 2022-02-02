# -*- coding: utf-8 -*-
import robot
import time

r = robot.Robot()
r.connect('/dev/ttyACM0', None)
try:
    # r.vacuum_on()
    # time.sleep(1)
    # r.vacuum_off()
    # input()

    # Инициализация при запуске робота
    #r.init()
    # Ждем кнопку RUN
    #r.wait_btn()
    # Прочитать код
    #code = r.read_code()
    #print(code)
    # r.to_storage()
    # r.find_circle()
    # r.take_ball()
    # r.rotate_180()
    # r.buhta()
    #r.spin_me_right_round()
    #r.find_circle()
    #r.set_pos({'j2':[ 135, 100, 100]})
    #r.vacuum_off()
    r.camera_set()
    #r.center()
    r.huy_v_zhopu()
    #r.set_pos({'j2':[ 90, 60, 100]})
    r.stop()

except:
    r.stop()
    raise Exception("Error!")

r.close()
