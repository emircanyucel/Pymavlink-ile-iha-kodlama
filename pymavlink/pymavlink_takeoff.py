'''
- Bu kodlar Emircan Yücel tarafından hazırlanmıştır.


- Pymavlink kütüphanesi ile İHA kodlama tecrübelerimi ve bilgilerimi paylaştığım bu ve bunun  gibi
kodların detaylı anlatımı için kendi hazırladığım öğretici içeriklerime aşağıdaki linkten ulaşabilirsiniz.
https://medium.com/@emircanyucel27

- İletişim için;
https://www.linkedin.com/in/emircan-y%C3%BCcel-267475246

'''


from pymavlink import mavutil

iha_baglanti = mavutil.mavlink_connection('udp:localhost:14550')
iha_baglanti.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (iha_baglanti.target_system, iha_baglanti.target_component))
iha_baglanti.mav.command_long_send(iha_baglanti.target_system, iha_baglanti.target_component,
                                   mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
msg = iha_baglanti.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)
iha_baglanti.mav.command_long_send(iha_baglanti.target_system, iha_baglanti.target_component,
                                   mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
msg = iha_baglanti.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)
