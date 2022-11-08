# Pymavlink ile iha kodlama
 Burada pymavlink yazılım kutuphanesi kullanarak iha'ya komut gonderme islemlerini yapacagiz.

---------------------------------
Pymavlink İHA Yaw ve Hız Kontrolü
---------------------------------

Burada pymavlink kütüphanesi kullanarak İHA’mızın yaw ve hız değerlerini değiştirmeyi göreceğiz.

Öncelikle İHA’nın hızını değiştirmeyi görelim. Bunu yapabilmek için kullandığımız mavlink mesajı MAV_CMD_DO_CHANGE_SPEED mesajıdır. Bu mesaj aracın hızını metre/saniye cinsinden arttırıp azaltmaya yarar. Tek parametre alır o da 2. parametre.

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
from pymavlink import mavutil

iha_baglanti = mavutil.mavlink_connection('udp:localhost:14550')
iha_baglanti.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (iha_baglanti.target_system, iha_baglanti.target_component))

iha_baglanti.mav.command_long_send(iha_baglanti.target_system, iha_baglanti.target_component,
                                   mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 10, 0, 0, 0, 0, 0, 0)
                                   '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
                                   
Buradan da anlaşılacağı üzere 2. parametreyi 10 yaptık ve artık aracımız 10 m/s hızla hareket edecek.

Şimdi de İHA’mızın yaw açısını değiştirmeyi deneyelim. Bunun için kullanmamız gereken mavlink mesajı MAV_CMD_CONDITION_YAW mesajıdır. Bu mesaj 4 adet parametre alır. Anlatmaya son parametreden başlayacağım çünkü bu parametre bu mesajı nasıl kullanacağımızı belirleyen parametre. Dördüncü parametrede 2 farklı girdi seçeneğimiz var. Eğer buraya 0 girersek iha 0 kuzey olacak şekilde girilen dereceye mutlak koordinatlara göre bakar. Fakat 1 girersek göreceli bir yöne bakma durumu oluşur yani biz ilk parametrede 45 derece dönmesini söylediğimizde o an nereye bakıyorsa baktığı yönü kuzey kabule ederek 45 derece sağa döner. Şimdi, ilk parametreye yaw açısı girilir. Bu girilen açının sonucu ise 4. parametreye göre belirlenir. İkinci parametre ise sapma değişimi sırasındaki hızı girdi olarak alır [derece/saniye]. Üçüncü parametre sadece göreli yani 4. parametreye 1 girildiği durumda çalışır ve -1 girilirse saat yönünün tersi; 1 girilirse saat yönünde dönülmesini sağlar.

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
from pymavlink import mavutil

iha_baglanti = mavutil.mavlink_connection('udp:localhost:14550')
iha_baglanti.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (iha_baglanti.target_system, iha_baglanti.target_component))

iha_baglanti.mav.command_long_send(iha_baglanti.target_system, iha_baglanti.target_component,
                                   mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, 45, 0, 0, 0, 0, 0, 0)
                                   '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
                                   
Yukarıdaki kod 4. parametreye 0 girildiği durumda mutlak koordinatlara göre 45 derece dönme işlemini yapabileceğimiz örnek koddur.

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
from pymavlink import mavutil

iha_baglanti = mavutil.mavlink_connection('udp:localhost:14550')
iha_baglanti.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (iha_baglanti.target_system, iha_baglanti.target_component))

iha_baglanti.mav.command_long_send(iha_baglanti.target_system, iha_baglanti.target_component,
                                   mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, 45, 25, -1, 1, 0, 0, 0)
                                  
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
Yukarıdaki kod ise saat yönünün tersi olacak şekilde sapma değişimi sırasındaki hız: 25 iken [derece/saniye] 45 derecelik yaw açısı oluşturmasını sağlar.

Gerekli linkler:

(https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html#mav-cmd-condition-yaw)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

---------------------------------
Pymavlink İHA Hareket Komutları
---------------------------------

Pymavlink kullanarak aracımıza hareket komutları gönderebilmekteyiz. Aracı hareket ettirmek için araca gönderebileceğimiz birçok mavlink mesajı bulunmaktadır. Biz bugün bunlardan 3 tanesini kullanacağız. Bunlar; SET_POSITION_TARGET_LOCAL_NED, SET_POSITION_TARGET_GLOBAL_INT, SET_ATTITUDE_TARGET mesajları.

Bu mesajları kullanmadan önce kısaca farklarından bahsetmek istiyorum.

SET_POSITION_TARGET_LOCAL_NED mesajı aracın hedef konumunu, hızı, ivmeyi, yönü veya dönüş hızını ayarlayabildiğimiz bir mavlink mesajıdır. Bu mesaj hareketin kalkış noktasını referans alarak yapar.
SET_POSITION_TARGET_GLOBAL_INT mesajı aracın hedef konumunu, hızı, yönü veya dönüş hızını ayarlamamıza olanak sağlar. SET_POSITION_TARGET_LOCAL_NED mesajına benzer (yukarıya bakın), ancak konumlar enlem ve boylam değerleri olarak sağlanır ve rakımlar, eve veya araziye göre deniz seviyesinin üzerinde olabilir.
SET_ATTITUDE_TARGET mesajı aracın hedef tutumunu ve tırmanma hızını veya itişini ayarlar.
Şimdi anlatırken dokümantasyondan adım adım takip edebilmeniz için öncelikle linkleri paylaşmak istiyorum.

(https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#set-position-target-local-ned)

(https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED)

Linklerden de görebileceğimiz üzere birçok girdi almaktadır bu mesajlar. Bunlardan ilki “time_boot_ms” parametresi. Denemelerimden anladığım kadarıyla bu parametre çok fazla etki göstermiyor bu sebeple ben genellikle buraya 10 giriyorum. Sonrasında bilidğimiz gibi target_system ve target_component parametreleri geliyor ve bunlar aracımızla bağlantı kurduğumuzda otomatik olarak dolduruluyor.

Sonrasında coordinate_frame parametresi geliyor ki bu parametre oldukça kritik bir parametre. coordinate_frame parametresi birkaç girdi seçeneği sunmaktadır. Bunlar;


parametreleridir. Ek olarak bunlara yukarıdaki linkten de ulaşabilirsiniz. Şimdi bu parametreler nedir?

İlk mesaj yani MAV_FRAME_LOCAL_NED mesajı diyor ki. Sen ihaya hareket komutu gönderdiğinde başlangıç noktası neresiyse orayı origin kabul et ve oranın da gerçekteki koordinat sistemini kullanarak (ihanın baktığı yerin kuzeyi güneyi değil de dünyanın kendi kuzey güneyini kullan) hareket et. Yani diyelim ki kuzeye 1 metre güneye 2 metre git dedik. Drone nerede olursa olsun kalkış noktasının dünyanın orjinal kuzey güney yönüne göre 1 metre kuzeye 2 metre güneye gider.
İkincisi yani MAV_FRAME_LOCAL_OFFSET_NED mesajında ise diyor ki; Bu sefer referans alacağın yer kalkış noktan değil, kendi bulunduğun konum. Yani eğer siz kuzeye 2 metre batıya 1 metre git derseniz iha kendi konumundan dünyanın orjinal kuzey güney yönüne göre kuzeye 2 metre batıya 1 metre gider.
Üçüncü parametre MAV_FRAME_BODY_NED ise yine kalkış noktasını referans alır fakat gidiş yönü dünyanın kendi orjinal kuzeyi güneyi değil, kendi baktığı yöne göredir. Yani ihanın baktığı yön kuzey olur, sağı doğu olur gibi…
Son mesaj ise MAV_FRAME_BODY_OFFSET_NED . Burada da kendi konumuna göre kendi yönüne göre hareket eder yani siz kuzeye 4 metre git derseniz iha neredeyse oradan başlar ve kendi baktığı yere göre karşısına doğru yani kendi kuzeyine 4 metre gider.
Bu bilgilere ek olarak eğer ihaya yukarı yönde bir komut gönderirseniz yani 10 metre yukarıya çıkmasını isterseniz bunu z koordinatını girdiğimiz kısımda eksi “-” ile göstermemiz gerek yani 10 metre yukarı çıkmak için -10; 10 metre aşağı inmesi için ise 10 girmeliyiz.

Biz burada deneme amaçlı ilk mesajı yani MAV_FRAME_LOCAL_NED i kullanacağız.

Bundan sonraki parametre type_mask. Aşağıda da gördüğünüz gibi birkaç farklı seçeneğimiz var seçmek için. Şimdi burada mantık şu. Bir altında x,y,z girdilerini görüyoruz. Bunların aktifliğini sağlamak için öncesinde type_mask girmeliyiz. Daha açık konuşmak gerekirse, biz bir parametrenin aktif olmasını istersek onu type_maskta 0; aktif olmamasını istersek onu type_maskta 1 olarak gireriz. type_masklar sağdan başlanarak yazılı ya da kontrol edilir. Örneğin ilkinde (use position) sağdan başlarsak başmaya ilk üçü 0 yani alttaki listede ilk üçü aktif sonrasındaki birler sırasıyla o parametrelerin pasif olduğunu gösterir.


Sonrasında gereken parametre listesi ise şu şekilde;


Burada da sırasıyla x,y,z parametrelerine metre cinsinden yönü belirtip gerisine default olarak kullanmayacağımız için daha doğrusu type maskta öyle girilmesinden dolayı 0 gireceğiz az sonraki örnekte. Eğer örneğin yaw açısını ayarlamak istersek o zaman sondan bir önceki parametreyi değiştirmemiz gerek. Mesela buraya 1.57 girersek (bu 90 dereceye tekabül ediyor) o zaman iha uçarken gittiği yönün 90 derece sağına bakarak ileri gider. Fakat bunu yapınca type_mask ı da değiştirmemiz gerek. Biz aşağıda 0b110111111000 kullanmıştık ve gördüğünüz gibi son iki parametre yani en solda 0b nin yanındaki 2 tane byte ı 1 demiştik çünkü yaw kullanmıyorduk ve de aktif ettik onları fakat eğer yaw parametresini aktif edeceksek yerine 0 yazmalıyız yani; 0b100111111000 olmalı. Yaw rate ise her saniye kaç radyan döneceğini belirtir.

Öğrendiğimiz bu bilgilerin ardından kodlama kısmına geçelim. Şimdiye kadar anladıklarımızı kod satırlarına döktüğümüzde şu şekilde olur.

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
from pymavlink import mavutil

iha_baglanti = mavutil.mavlink_connection('udp:localhost:14550')
iha_baglanti.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (iha_baglanti.target_system, iha_baglanti.target_component))

iha_baglanti.mav.send(mavutil.mavlink.MAVLink_set_pozition_target_local_ned_message(10, iha_baglanti.target_system,
                                                                                    iha_baglanti.target_component,
                                                                                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                                                    int(0b110111111000), 10, 0, -10, 0,
                                                                                    0, 0, 0, 0, 0, 0, 0))
Daha sonra ihanın anlık konumunun takibi için NAV_CONTROLLER_OUTPUT mesajını ya da LOCAL_POSITION_NED çekerek bir döngü oluşturabiliriz.

while True:
    msg = iha_baglanti.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
    print(msg)
    time.sleep(1)
    
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

-------------------------------------------
Pymavlink ile İHA’yı Havalandırma (TAKEOFF)
-------------------------------------------

Burada aracımızı havalandırmak için kullanmamız gereken mavlink mesajı MAV_CMD_NAV_TAKEOFF mesajıdır.

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
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
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Burada öncelikle pymavlink kütüphanemizi import ettik ve kullanacağımız mavutil fonksiyonunu çektik. Daha sonra önceki anlatımlardan öğrendiğimiz şekilde gerekli bağlantıları yaptık ve İHA’mızı arm durumuna getirdik. Şimdi burada takeoff işlemi için öncelikle mavlink mesajımızı COMMAND_LONG dan çekeceğimizi bilmemiz gerek. Ardından bildiğimiz COMMAND_LONG parametrelerini girmeye başlıyoruz. Target system ve target component parametrelerini girdikten sonra yapmak istediğimiz takeoff işlemi için MAV_CMD_NAV_TAKEOFF mavlink mesajını çekiyoruz. Bu mesaj 7 adet parametre alır. Birincisi hatve (pitch) için kullanılan parametre. Bu parametre sadece sabit kanatlar için geçerli. Eğer sabit kanat kullanıyorsanız buraya 1 girmeniz gerek fakat ben şimdi dönerkanat için yazdığım için buraya 0 giriyorum. Siz de eğer dönerkanat kullanıyorsanız buraya 0 girmelisiniz. Dökümantasyonda ikinci ve üçüncü parametrelerin boş bıkarılması gerektiği söylenmekte, bu sebeple buralara 0 giriyoruz. Dördüncü parametreyi ise yine dönerkanat kullanmamız sebebiyle 0 giriyoruz ve geriye kaldı 3 parametre. Bunlardan ilki Latitude ikincisi Longitude, sonuncusu ise Altitude. Latitude ve longitude değerlerini 0 girersek olduğu yerden altitude parametresinde girdiğimiz irtifaya kadar yükselir. Biz de örnekte böyle yaptık.

Daha sonra COMMAND_ACK mesajını çektik ve sonucu döndürdük.

(https://mavlink.io/en/messages/common.html#MAV_RESULT)

(https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html#mav-cmd-nav-takeoff)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

-------------------------------------------
Pymavlink ile İHA’yı Arm Durumuna Getirme
-------------------------------------------

Öncelikle İHA’yı arm etme komutlarına geçmeden önce sizlere komut protokolünden kısaca bahsetmek istiyorum. Komut protokolü aşağıdaki şemadan da görüleceği üzere çokta karmaşık olmayan bir işleme sahip. Öncelikle bir komut gönderdiğimizde GCS ya da kullandığınız program İHA’ya COMMAND_LONG mesajı gönderir. İHA komutu aldıktan sonra komutun içerdiği mesajı işleme alır ve comut içeriğine göre aksiyon gösterir. Eğer aksiyon veya bilgi başarılı/başarısız olduğunda bu bilgiyi COMMAND_ACK mesajında paketler ve GCS ye geri gönderir. Gönderilen bu bilgiyi GCS ya da kullanıcı yorumlar ve çıktıya göre işlem sağlanır. Bu döngü işlem sonlanana kadar devam eder.


Şimdi COMMAND_LONG kullanımına bakalım.
COMMAND_LONG mesajı birçok parametre alıyor. Bunlardan ilk ikisi target system ile component system. Bunlar sabit olarak her mesaj içeriğinde yazılmalıdır. Sonra kullanacağımız mavlink mesajının adını girmemiz gerekli. Bu adımdan sonraki parametre onaylama parametresidir ve default olarak 0 girilir. Bundan sonraki 7 parametre, girdiğimiz mavlink mesajına ait olacak parametreler. Mesela bu komut ilk iki parametreyi dolduruyor başka parametre almıyor bu yüzden gerisini 0 girdik. Bu kullandığımız mavlink mesajı İHA’mızı arm durumuna getirmeye yarıyor. Bu ilk iki parametreden ilkinde 1 girersek arm ederken; 0 girersek disarm eder. Mavlink mesajlarının detaylı incelemesi ve tüm dökümantasyon için aşağıdaki linklerden yararlanabilirsiniz.

(https://mavlink.io/en/services/command.html https://mavlink.io/en/messages/common.html#COMMAND_LONG https://mavlink.io/en/messages/common.html#MAV_CMD )

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
from pymavlink import mavutil

iha_baglanti = mavutil.mavlink_connection('udp:localhost:14550')

iha_baglanti.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (iha_baglanti.target_system, iha_baglanti.target_component))

iha_baglanti.mav.command_long_send(iha_baglanti.target_system, iha_baglanti.target_component,
                                   mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
Buraya kadar COMMAND_LONG mesajı ile İHA’mıza mesaj gönderdik fakat yukarıda da bahsettiğim gibi döngünün tamamlanması için her mesajın bir COMMAND_ACK gerbildirim mesajına ihtiyacı var. Şimdi bu mesajı oluşturalım.

msg = iha_baglanti.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)
Burada daha önceden öğrendiğimiz mesaj çekme yöntemi ile COMMAND_ACK mesajı oluşturduk. Bu mesajın çıktısı ise yaklaşık olarak şu şekilde olmalı.

Heartbeat from system (system 1 component 0)
COMMAND_ACK {Command : 400, result : 0, progress : 0, result_param2 : 0, target_system : 0, target_component : 0}
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Burada bilememiz gereken şey şu:

Farkettiyseniz çıktıda result isimli bir çıktımız var ve 0 gösteriyor. Peki bu ne anlama gelmektedir? Eğer dökümantasyonda COMMAND_ACK mesajının içeriğini incelersek;


2. sıradaki çıktının içeriğini bize verdiğini görebiliriz. Peki, şimdi bu MAV_RESULT içeriğini inceleyelim. Bakalım bize çıktı olarak verilen 0 ne anlama geliyormuş.


Gördüğümüz üzere aslında bize verilen çıktının gönderdiğimiz mavlink mesajının geçerli olduğunu ve işleme alındığını söylemektedir.

Bu kısımda İHA’mızı nasıl arm edeceğimizi ve ek olarak COMMAND_LONG ve COMMAND_ACK mesajlarını nasıl kullanacağımızı gördük. Gerekli linkleri aşağıda bulabilirsiniz.

(https://mavlink.io/en/messages/common.html#COMMAND_ACK)

(https://mavlink.io/en/messages/common.html#MAV_RESULT)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

----------------------------------------------------
Pymavlink ile İHA’ya Bağlanma ve Durum Bilgisi Çekme
----------------------------------------------------


Öncelikle kısaca pymavlinkten bahsetmek istiyorum. Pymavlink, Python ile yazılmış, genel amaçlı bir MAVLink mesaj işleme kütüphanesidir. Bir GCS (MAVProxy), Geliştirici API’leri (DroneKit) ve çok sayıda yardımcı bilgisayar MAVLink uygulaması dahil olmak üzere birçok MAVLink sisteminde MAVLink iletişimini uygulamak için kullanılmıştır.

Pymavlink kütüphanesini kullanmadan önce elbette ki kurulumunu yapmamız gereklidir.

pip install pymavlink
Komut istemini açıp yukarıdaki bloğu yapıştırdığımızda pymavlink kütüphanesinin kurulumunu kolaylıkla yapabiliriz.

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
from pymavlink import mavutil 
import time 
iha_baglanti = mavutil.mavlink_connection('udp:localhost:14550')
iha_baglanti.wait_heartbeat() 
print("Heartbeat from system (system %u component %u)" % (iha_baglanti.target_system, iha_baglanti.target_component)) 
while True: 
    msg = iha_baglanti.recv_match(blocking=True) 
    print(msg) 
    time.sleep(1)
Şimdi kodlarımızı yazmaya başlayalım. Öncelikle pymavlink kütüphanesini import etmemiz gerekli. Pymavlink kütüphanesinden ihtiyacımız olan fonksiyonları daha sonra çekeceğiz. Örneğin ilk olarak mavutil modülünü çekiyoruz. Mavutil, pymavlink kütüphanesinde bağlantıları veya mesaj göndermeleri sağlayan önemli bir modül. Şimdi burada kütüphaneleri import ettikten sonra iha_baglanti isimli değişken oluşturduk ve bu değişkende mavutilden mavlink_connection fonksiyonunu kullanarak bağlantıyı sağladık. Localhostta 14540 bağlantısını gerçek hayatta kullanıyoruz fakat yukarıdaki örnekte sitl kullanacağımız için 14551 yazıyoruz. Her iha sürekli yer istasyonu yazılımlarına anlık durum mesajı gönderir (bilgilerini nerde olduğunu vs). Bu bilgileri wait_heartbeat() ile çekiyoruz. Daha sonra msg adında değişken oluşturduk. Burada recv_match fonksiyonu tüm mesajların kullanılabilir olmasını sağlıyor. Şimdi burada bu msg değişkenini yazdırınca drone ile ilgili birçok bilgi ekrana yansıtılacak. Bu bilgilerden ihtiyacımız olanları çekmek istersek şunu yapıyoruz.

msg = iha_baglanti.recv_match(type='ALTITUDE', blocking=True)
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Örneğin buarada sadece ALTITUDE bilgisini çektik.

İşlemin sonunda buna benzer çıktı almamız gerekmektedir.

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
"""
Heartbeat from system (system 1 component 0)
ATTITUDE {time_boot_ms : 288681, roll : -0.000892952783033, pitch : 0.00105964264367, yaw : 0.017788624391, rollspeed : 0.000322287902236, pitchspeed : 0.000295313308015, yawspeed : 0.000818725209683}
GLOBAL_POSITION_INT {time_boot_ms : 288681, lat : -353632622, lon : 1491652375, alt : 583970, relative_alt : -19, vx : -1, vy : 1, vz : 0, hdg : 101}
SYS_STATUS {onboard_control_sensors_present : 1399979183, onboard_control_sensors_enabled : 1382128815, onboard_control_sensors_health : 1399954607, load : 0, voltage_battery : 12587, current_battery : 0, battery_remaining : 100, drop_rate_comm : 0, errors_comm : 0, errors_count1 : 0, errors_count2 : 0, errors_count3 : 0, errors_count4 : 0}
POWER_STATUS {Vcc : 5000, Vservo : 0, flags : 0}
MEMINFO {brkval : 0, freemem : 65535, freemem32 : 131072}
NAV_CONTROLLER_OUTPUT {nav_roll : -0.000111882312922, nav_pitch : 0.000109248649096, nav_bearing : 1, target_bearing : 0, wp_dist : 0, alt_error : -0.000154292603838, aspd_error : 0.0, xtrack_error : 0.0}
MISSION_CURRENT {seq : 0}
VFR_HUD {airspeed : 0.0, groundspeed : 0.0251470748335, heading : 1, throttle : 0, alt : 583.969970703, climb : -1.06446605059e-05}
SERVO_OUTPUT_RAW {time_usec : 288681099, port : 0, servo1_raw : 1000, servo2_raw : 1000, servo3_raw : 1000, servo4_raw : 1000, servo5_raw : 0, servo6_raw : 0, servo7_raw : 0, servo8_raw : 0, servo9_raw : 0, servo10_raw : 0, servo11_raw : 0, servo12_raw : 0, servo13_raw : 0, servo14_raw : 0, servo15_raw : 0, servo16_raw : 0}
RC_CHANNELS {time_boot_ms : 288681, chancount : 16, chan1_raw : 1500, chan2_raw : 1500, chan3_raw : 1000, chan4_raw : 1500, chan5_raw : 1800, chan6_raw : 1000, chan7_raw : 1000, chan8_raw : 1800, chan9_raw : 0, chan10_raw : 0, chan11_raw : 0, chan12_raw : 0, chan13_raw : 0, chan14_raw : 0, chan15_raw : 0, chan16_raw : 0, chan17_raw : 0, chan18_raw : 0, rssi : 255}
RAW_IMU {time_usec : 288681099, xacc : 0, yacc : 1, zacc : -1000, xgyro : 3, ygyro : 3, zgyro : 3, xmag : 232, ymag : 53, zmag : -528, id : 0, temperature : 3671}
SCALED_IMU2 {time_boot_ms : 288681, xacc : 0, yacc : 1, zacc : -1000, xgyro : 4, ygyro : 3, zgyro : 4, xmag : 232, ymag : 53, zmag : -528, temperature : 3671}
SCALED_IMU3 {time_boot_ms : 288681, xacc : 0, yacc : 0, zacc : 0, xgyro : 0, ygyro : 0, zgyro : 0, xmag : 232, ymag : 53, zmag : -528, temperature : 0}
SCALED_PRESSURE {time_boot_ms : 288681, press_abs : 945.039367676, press_diff : 0.0, temperature : 3120, temperature_press_diff : 0}
SCALED_PRESSURE2 {time_boot_ms : 288681, press_abs : 945.042114258, press_diff : 0.0, temperature : 3120, temperature_press_diff : 0}
GPS_RAW_INT {time_usec : 288565000, fix_type : 6, lat : -353632621, lon : 1491652374, alt : 583990, eph : 121, epv : 200, vel : 0, cog : 18189, satellites_visible : 10, alt_ellipsoid : 0, h_acc : 300, v_acc : 300, vel_acc : 40, hdg_acc : 0, yaw : 0}
SYSTEM_TIME {time_unix_usec : 1664897947515841, time_boot_ms : 288681}
AHRS {omegaIx : -0.00352771673352, omegaIy : -0.00355346896686, omegaIz : -0.00299386004917, accel_weight : 0.0, renorm_val : 0.0, error_rp : 0.00246560410596, error_yaw : 0.00147348584142}
SIMSTATE {roll : -0.0021275926847, pitch : -5.26245145238e-08, yaw : 4.49366780231e-06, xacc : 0.0282704941928, yacc : 0.0331453196704, zacc : -9.79741573334, xgyro : 4.04270649597e-05, ygyro : -6.97283321642e-05, zgyro : -6.39353456791e-05, lat : -353632621, lng : 1491652374}
AHRS2 {roll : -0.000325416476699, pitch : 0.00169665017165, yaw : -0.00323487026617, altitude : 583.989990234, lat : -353632621, lng : 1491652374}
HWSTATUS {Vcc : 5000, I2Cerr : 0}
TERRAIN_REPORT {lat : -353632622, lon : 1491652375, spacing : 100, terrain_height : 583.84375, current_height : -0.0198779292405, pending : 0, loaded : 504}
EKF_STATUS_REPORT {flags : 831, velocity_variance : 0.00919852033257, pos_horiz_variance : 0.00533722667024, pos_vert_variance : 1.54203753482e-05, compass_variance : 0.0137246921659, terrain_alt_variance : 0.0, airspeed_variance : 0.0}
LOCAL_POSITION_NED {time_boot_ms : 288681, x : -0.0147634167224, y : 0.0131693389267, z : -0.000154292603838, vx : -0.0187205001712, vy : 0.0167904216796, vz : 1.06446605059e-05}
VIBRATION {time_usec : 288681099, vibration_x : 0.0349595025182, vibration_y : 0.00951889902353, vibration_z : 0.0033915725071, clipping_0 : 0, clipping_1 : 0, clipping_2 : 0}
BATTERY_STATUS {id : 0, battery_function : 0, type : 0, temperature : 32767, voltages : [12587, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535], current_battery : 0, current_consumed : 0, energy_consumed : 0, battery_remaining : 100, time_remaining : 0, charge_state : 1, voltages_ext : [0, 0, 0, 0], mode : 0, fault_bitmask : 0}

"""
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
