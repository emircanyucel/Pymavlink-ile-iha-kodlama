'''
- Bu kodlar Emircan Yücel tarafından hazırlanmıştır.


- Pymavlink kütüphanesi ile İHA kodlama tecrübelerimi ve bilgilerimi paylaştığım bu ve bunun  gibi
kodların detaylı anlatımı için kendi hazırladığım öğretici içeriklerime aşağıdaki linkten ulaşabilirsiniz.
https://medium.com/@emircanyucel27

- İletişim için;
https://www.linkedin.com/in/emircan-y%C3%BCcel-267475246

'''

import time

from pymavlink import mavutil

iha_baglanti = mavutil.mavlink_connection('udp:localhost:14550')
iha_baglanti.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (iha_baglanti.target_system, iha_baglanti.target_component))


iha_baglanti.mav.send(mavutil.mavlink.MAVLink_set_pozition_target_local_ned_message(10, iha_baglanti.target_system,
                                                                                    iha_baglanti.target_component,
                                                                                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                                                    int(0b110111111000), 10, 0, -10, 0,
                                                                                    0, 0, 0, 0, 0, 0, 0))
while True:
    msg = iha_baglanti.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
    print(msg)
    time.sleep(1)
