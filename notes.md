- 1600Hz 3-axis accelerometer
- 1600Hz 3-axis gyroscope
- 30Hz magnetometer (tusim viac ani na tom arduine nemozeme)
- 50Hz of barometer (vyska, tlak, teplota)
- 3Hz of GPS

reality
- 100Hz Accel + Gyro 
- 10Hz Mag
- 20Hz Baro
- 3Hz GPS

https://github.com/SparkyVT/HPR-Rocket-Flight-Computer



takze hovoris, ze budem mat 1600 merani za sekundu a v ramci toho jedneho meracieho zaznamu moze ale nemusi byt ten magnetometer / barometer atd hej 
a ked to budem zapisovat tak to len zapisem ako nejake byty

timestamp + accelerometer + gyroscope + ?magnetometer + ?barometer + ?gps |
timestamp + accelerometer + gyroscope + ?magnetometer + ?barometer + ?gps | 
...
az dokym nemam dostatocne vela a ked mam tak to zapisem buffer uvolnim a znovu pokracujem hej




Raketa (Iris) -> Hermes (Ground station)
Raketa po odpale zacne posielat velmi velmi obmedzene telemetry data, konkretnejsie raz za kazdu sekundu posle momentalnu vysku, momentalnu rychlost, a pod. Tbh len sem jebni nejake duimmy values, nejde o to ake data ale ze bude proste posielat kazdu sekundu len nejaky snapshot toho ako vysoko je, ako rychla je v tom momente a pod.

----

Raketa (Iris) -> Hermes (Ground station)
Uplne na zaciatku este ako je raketa na zemi a este pred tym ako zacne posielat update pakety (hned prva polozka v tom subore paketov) bude raketa posielat nejaky "parovaci" paket. Napr kazdych 5 sekund posle lora paket ze hej je tu nejaka base station. Ak dostane Ack raketa caka napr 30 sekund na doparovanie (vid dalsi paket). Ak za 30 sekund nedoparuje zacne zas posielat kazdych 5sekund

Hermes (Ground station) -> Raketa (Iris)
Ground stanica odpovie s identifikatorom rakety (toto je len jednoduchy byte, napr 0,1,2,3...). Toto IDcko musi byt vo vsetkych paketoch aby sme kvazi vedeli ze komunikujeme prave s touto raketou (hej ja viem, haha hihi, viac ako jedna urcite. Ale tak jeden byte hore dole). Ked prebehne tento krok je parovanie hotove a zacnu sa posielat dalsie pakety co uz su popisane v tom subore

Ak teda mame raketu (iris) a grouind station (hermes), tak... 

Raketa (Iris) -> Hermes (Ground station)
Ked je raketa na zemi, posiela ground station update packety (toto moze byt nejaka jedna struktura) -> senzory su ready, gps lock acquired, proste ready status, aby sme raketu odpalili az ked naozaj je ready. Frekvencia posielania zalezi na statuse, resp. posles update packet vtedy ked gps dostane lock, posles packet ked gps strati lock. Netreba to v nejakom intervale, len on change. 

Raketa (Iris) -> Hermes (Ground station)
Ked raketa zdetekuje ze bola odpalena posle jeden paket so zmenou stavu (launched). V tomto momente zacnes posielat pravidelne aj GPS poziciu, zalezi na sampling rate, ale uprimne stacia tak 1 paket kazdych 0.5s.

Raketa (Iris) -> Hermes (Ground station)
Ked raketa hocikedy po odpale strati GPS lock informuje o tom base station, rovnako ak ho znovu re-acquirne.

Raketa (Iris) -> Hermes (Ground station)
Raketa po odpale zacne posielat velmi velmi obmedzene telemetry data, konkretnejsie raz za kazdu sekundu posle momentalnu vysku, momentalnu rychlost, a pod. Tbh len sem jebni nejake duimmy values, nejde o to ake data ale ze bude proste posielat kazdu sekundu len nejaky snapshot toho ako vysoko je, ako rychla je v tom momente a pod.

Raketa (Iris) -> Hermes (Ground station)
Ked raketa zdetekuje ze dovrsila apogee (max dovrsenu vysku), posle zoznam vyhodnotenych statistik -> max dvrsena vyska, max vertikalna rychlost, max Gcka a pod a zmeni status na "freefall" alebo daco take

Raketa (Iris) -> Hermes (Ground station)
Ked raketa zdetekuje ze jebla na zem posle svoju finalnu lokaciu a zmena statusu na "landed". Toto most likely failne lebo lora nebude mat line of sight , ale ak nie je to insta win. Mozes to spravit tak ze napr ked uz bude jebnuta na zemi budes posielat tento landed status dookola kazdych 30 sekund

Hermes (Ground station) -> Raketa (Iris)
Nechcem to tuto velmi komplikovat, kedze uz nemame vela casu tak tu prosim ta len sprav nejaku PoC "Status" funkciu. Ze hermes moze tej rakete poslat hocikedy nejaky get status paket a raketa odpovie s datami ako gps poloha a vsetky data so senzorov v tom momente (nebude problem s bandwidthom lebo berieme len data v tom momente, jeden sample kvazi)

A pri vsetkych paketoch predpokladajme ze za kazdym co nedostaneme ACK na spravu sa sprava posle znovu 3x, potom spravu uz dropneme. V radiach povol automaticke CRC a podobne kktiny nech nemusime robit vlasny error correct, myslim uprimne ze tam mas aj tento retransmittion tak treba vyskusat. Taktiez povol RSSI ktore budes posielat v kazdom pakete aby sme vedeli aka je sila spojenia. Mozes nastavit aj nejake sifrovane (to je dost easy to som skusal len nastavis 2 registre). Taktiez v kazdom pakete by malo byt nejake idcko tej rakety a taktiez idcko base stationu.

Dolezite je taktiez nastavit veci ako spread factor a podobne v tych radiach, proste zarucit dobry bandwidth ale zaroven distance dobry. Kedze mi nebudeme velmi saturovat tu linku (takmer vobec, lol), tak mozeme asi prioritizovat vacsi distance ako vyssi bandwidth.



-----

@Kono ty by si mohol zacat uz programovat nejaku logiku tej rakety ako takej. Resp, overit si ci vieme tahat data so vsetkych senzorov, rozhodnut sa ako budes tie data logovat na sd kartu (asi csv format? -> toto je priorta, lebo dominikovi je treba tento format pre jeho vyvoj), ako rychlo samplovat tie senzory (napis mi este ohladom toho) a pomalicky by si mohol zacat pracovat na tej state machine, napr detekovanie odpalu, detekovanie max vysky a pod. Odporucam od cca 18tej minuty pozriet https://youtu.be/4cw9K9yuIyU?t=1088 , cca do 30tej. Pripadne je vela zdrojov na internete, si vyhladaj napr "model rocket flight computer github" a podobne, https://github.com/SparkyVT/HPR-Rocket-Flight-Computer . Tu vies aj vytiahnut ako napr kalibrovat tie senzory a podobne