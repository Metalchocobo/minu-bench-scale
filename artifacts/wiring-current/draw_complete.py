"""Generate one complete assembly map using Andrea's actual ESP32 pin layout.

Only the ESP32 header positions are physical. Other terminal positions are
functional and retain their names. Every documented external net is drawn.
"""
from pathlib import Path
from html import escape
import sys
sys.dont_write_bytecode = True
from draw_wiring import Plate, supply_capacitors, RED, INK, BLUE, GREEN, ORANGE, PAPER, MUTED, UNKNOWN

ROOT = Path(__file__).resolve().parent
WIDTH, HEIGHT = 3200, 2250
AUDIO_POWER = '#9e3655'


class Complete(Plate):
    def __init__(self):
        self.number = 0
        self.wires, self.parts, self.nodes, self.panels = [], [], [], []
        self.rect(0, 0, WIDTH, HEIGHT, PAPER, PAPER, 0)
        self.text(70, 58, 'MINÙ / CABLAGGIO · SCHEMI FORNITI + DOCUMENTAZIONE HX711', 26, GREEN, True)
        self.text(70, 120, 'Tutti i fili, con i pin dell’ESP32 nella posizione della tua mappa', 45, INK, True)
        self.text(70, 171, 'Un solo foglio A2 · Leggi i valori delle resistenze · Incrocio senza pallino = fili separati', 28, MUTED)

    def save(self, stem):
        background, foreground = self.parts[:1], self.parts[1:]
        svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" height="{HEIGHT}" viewBox="0 0 {WIDTH} {HEIGHT}" role="img">',
               '<title>Minù: tutti i collegamenti e disposizione ESP32 a 30 pin</title>',
               '<desc>Ogni collegamento documentato è disegnato. L’ordine fisico dei pin ESP32 segue la mappa fornita da Andrea; gli altri moduli sono disposti per funzione. Le incognite fisiche sono indicate.</desc>']
        (ROOT/f'{stem}.svg').write_text('\n'.join(svg+background+self.panels+self.wires+foreground+self.nodes+['</svg>']), encoding='utf-8')


def cap_can(d, x, label, voltage, positive_node):
    """Draw an actual two-lead can: positive wraps around its left side."""
    top, bottom = 505, 635
    d.rect(x-50, top, 100, bottom-top, '#354b5a', '#263b48', 8)
    d.parts.append(f'<ellipse cx="{x}" cy="{top}" rx="50" ry="12" fill="#bdc9d0" stroke="#728895" stroke-width="2"/>')
    d.rect(x+20,top+4,16,126,'#c9d5dc','#c9d5dc',0)
    d.text(x-6,553,label,21,'white',True,'middle')
    d.text(x-6,583,voltage,20,'white',False,'middle')
    d.text(x+27,617,'−',22,INK,True,'middle')
    d.wire([positive_node,(x-75,480),(x-75,650),(x-30,650),(x-30,bottom)],RED)
    d.wire([(x+27,bottom),(x+27,690)],INK)
    d.text(x-15,663,'+',21,RED,True)
    d.dot(*positive_node,RED);d.dot(x+27,690,INK)


def cap_disc(d, x, positive_node):
    d.circle(x,580,35,'#c88839','#9d672b')
    d.text(x,589,'104',21,'white',True,'middle')
    d.wire([positive_node,(x-40,480),(x-40,650),(x-15,650),(x-15,611)],RED)
    d.wire([(x+15,611),(x+15,690)],INK)
    d.dot(*positive_node,RED);d.dot(x+15,690,INK)


def draw():
    d=Complete()
    # Common rails are outside module silhouettes, with a dot at each branch.
    d.wire([(520,230),(3080,230)],RED,7)
    d.wire([(520,230),(520,2030)],RED,7)
    d.wire([(3080,230),(3080,2030)],RED,7)
    d.dot(520,230,RED);d.dot(3080,230,RED)
    d.text(540,214,'5 V nominali · Mini360 regolato a 5,11 V sul montaggio',27,RED,True)
    d.wire([(1210,590),(1230,590),(1230,275),(3120,275),(3120,1660),(3040,1660)],ORANGE,7)
    d.dot(1210,590,ORANGE)
    d.text(2620,326,'3V3 ESP32 → OLED + VCC INA219',23,ORANGE,True)
    d.wire([(65,500),(65,2180),(3140,2180),(3140,490)],INK,7)
    d.wire([(65,690),(2450,690),(2450,500),(3140,500)],INK,7)
    d.dot(65,690,INK);d.dot(3140,500,INK)
    d.text(2710,2161,'GND / negativo comune',26,INK,True)

    # Battery -> fuse -> high-side current monitor -> switch -> buck.
    d.board(100,320,300,250,'NASTIMA','LiFePO4 · 6,4 V / 6 Ah','#dde8ef')
    d.text(250,490,'USB-C sul pacco',22,INK,True,'middle')
    d.text(250,530,'Ricarica esterna 5 V',21,INK,False,'middle')
    d.port(400,430,'+',RED);d.port(100,500,'−',INK,'left')
    d.wire([(100,500),(65,500)],INK)
    d.wire([(400,430),(540,430)],RED)
    d.rect(540,406,130,48,'#ecdabe','#b4a17e',8)
    d.text(605,439,'F1',26,INK,True,'middle')
    d.text(605,388,'Valore ignoto',22,UNKNOWN,True,'middle')
    d.wire([(670,430),(850,430)],RED)
    d.board(850,300,350,280,'INA219','VIN− è positivo, non GND')
    d.port(850,430,'VIN+',RED,'left');d.port(1200,430,'VIN−',RED)
    for x,label,color in [(900,'VCC',ORANGE),(960,'GND',INK),(1020,'SDA',BLUE),(1080,'SCL',BLUE)]:
        d.port(x,580,color=color)
        d.text(x,550,label,22,color,True,'middle')
    d.wire([(960,580),(960,690)],INK);d.dot(960,690,INK)
    d.wire([(1200,430),(1310,430)],RED)
    d.rect(1310,350,180,100,'#e8edf0','#a1afb8',9)
    d.rect(1380,370,45,55,'#35434c','#35434c',5)
    d.text(1400,328,'S1 · ON/OFF',26,INK,True,'middle')
    d.wire([(1490,430),(1580,430)],RED)
    d.board(1580,300,400,280,'Mini360','Terminali funzionali, non vista pad')
    for x,y,label,color,side in [(1580,430,'IN+',RED,'left'),(1580,530,'IN−',INK,'left'),(1980,430,'OUT+',RED,'right'),(1980,530,'OUT−',INK,'right')]:d.port(x,y,label,color,side)
    d.wire([(1580,530),(1520,530),(1520,690)],INK);d.dot(1520,690,INK)
    d.wire([(1980,530),(2000,530),(2000,690)],INK);d.dot(2000,690,INK)
    d.wire([(1980,430),(2030,430),(2030,230)],RED);d.dot(2030,230,RED)
    d.wire([(1540,430),(1540,480),(1285,480)],RED);d.dot(1540,430,RED)
    cap_can(d,1360,'220 µF','16 V',(1285,480))
    cap_disc(d,1480,(1440,480))
    d.wire([(2030,430),(2030,480),(2280,480)],RED);d.dot(2030,430,RED)
    cap_can(d,2150,'470 µF','10 V',(2075,480))
    cap_disc(d,2320,(2280,480))
    d.text(1780,485,'104 = 100 nF',24,INK,True,'middle')

    # Red LED, the actual series resistor remains unknown.
    d.parts.append('<path d="M2525 460V415 A45 45 0 0 1 2615 415V460Z" fill="#d84d46" stroke="#a9312e" stroke-width="3"/>')
    d.rect(2518,455,104,12,'#bb3834','#bb3834',4)
    d.parts.append('<path d="M2550 467V490 M2610 467V490" stroke="#899ba6" stroke-width="6"/>')
    d.port(2550,490,color=BLUE);d.port(2610,490,color=INK)
    d.text(2660,396,'LED rosso sleep',28,INK,True)
    d.text(2660,434,'Anodo + dal GPIO15',23)
    d.text(2660,471,'Catodo − a GND',23)
    d.wire([(2610,490),(3140,490)],INK);d.dot(3140,490,INK)
    d.resistor(2470,410,'1 kΩ LED',True,BLUE,'left')
    d.wire([(2470,485),(2470,490),(2550,490)])

    # ESP32: only here pin order/positions follow Andrea's supplied map.
    left=['3V3','GND','GPIO15','GPIO2','GPIO4','RX2 / 16','TX2 / 17','GPIO5','GPIO18','GPIO19','GPIO21','RX0 / 3','TX0 / 1','GPIO22','GPIO23']
    right=['VIN / 5V','GND','GPIO13','GPIO12','GPIO14','GPIO27','GPIO26','GPIO25','GPIO33','GPIO32','GPIO35','GPIO34','GPIO39 / VN','GPIO36 / VP','EN']
    d.rect(1380,775,430,850,'#dce9fc','#8eadd2',35)
    d.text(1595,1090,'ESP32',44,INK,True,'middle')
    d.text(1595,1140,'30 pin · 15 per lato',27,INK,True,'middle')
    d.text(1595,1180,'Ordine della mappa di Andrea',23,INK,False,'middle')
    ports={}
    unused={'RX0 / 3','TX0 / 1','GPIO12','GPIO36 / VP','EN'}
    for row,(l,r) in enumerate(zip(left,right)):
        y=810+55*row
        for x,label,is_left in [(1240,l,True),(1810,r,False)]:
            color=INK if label=='GND' else ORANGE if label=='3V3' else RED if label=='VIN / 5V' else BLUE
            fill='#e5e9ed' if label in unused else '#fff2c9'
            d.rect(x,y-19,140,38,fill,'#ccb980' if label not in unused else '#bdc6cd',2)
            d.text(x+70,y+8,label,22,MUTED if label in unused else color,True,'middle')
            edge=1240 if is_left else 1950
            if label not in unused:d.port(edge,y,color=color)
            ports[label]=(edge,y)
    d.text(1595,1460,'Pin grigi: nessun filo esterno',24,MUTED,False,'middle')
    d.text(1595,1510,'SDA21 / SCL22 sono alias:',22,INK,False,'middle')
    d.text(1595,1550,'INA219 usa GPIO32 / GPIO33',23,INK,True,'middle')
    # Power and INA signal nets.
    d.wire([ports['VIN / 5V'],(2480,810),(2480,910),(3080,910)],RED);d.dot(3080,910,RED)
    d.wire([ports['GND'],(2480,865),(2480,1090),(3140,1090)],INK);d.dot(3140,1090,INK)
    # The custom PCB pair is additional to the separate pair at the ESP32.
    d.rect(2500,505,530,295,'#edf5f1','#8cab9b',background=True)
    d.text(2525,546,'PCB custom · tra Mini360 e ESP32',24,INK,True)
    pcb_top,pcb_ground=supply_capacitors(d,2635,635,spacing=230,capacity='47 µF',voltage='16 V',dashed=False)
    d.wire([pcb_top,(3080,600)],RED,4);d.dot(3080,600,RED)
    d.wire([pcb_ground,(3140,780)],INK,4);d.dot(3140,780,INK)
    d.text(2525,798,'47 µF / 16 V + 104 · coppia aggiuntiva montata',18,INK,True)
    # External parts are installed; their shown values are recommended for replication.
    d.rect(2500,825,530,300,'#fff7e8','#dfc79a',background=True)
    d.text(2525,869,'Condensatori presso ESP32',25,INK,True)
    esp_top,esp_ground=supply_capacitors(d,2635,945,spacing=230,capacity='47 µF',voltage='16 V',dashed=False)
    d.wire([(esp_top[0],910),(2480,910)],RED,4);d.dot(2480,910,RED)
    d.wire([esp_ground,(3140,1090)],INK,4)
    d.text(2525,1114,'Valori consigliati; montati da leggere.',21,UNKNOWN,True)
    # Both headers have a GND pin; draw the left one explicitly as well.
    d.wire([(1240,865),(1200,865),(1200,690)],INK);d.dot(1200,690,INK)
    d.wire([ports['3V3'],(1210,810),(1210,590),(900,590),(900,580)],ORANGE)
    d.wire([ports['GPIO33'],(2330,1250),(2330,665),(1080,665),(1080,580)])
    d.wire([ports['GPIO32'],(2300,1305),(2300,680),(1020,680),(1020,580)])
    d.wire([ports['GPIO15'],(1120,920),(1120,605),(800,605),(800,260),(2470,260),(2470,335)])

    # DFPlayer, UART, BUSY, switched supply and the floating speaker output.
    d.board(100,750,330,340,'DFPlayer Mini','RX / TX / BUSY / SPK')
    for y,label,color in [(870,'VCC',AUDIO_POWER),(920,'RX',BLUE),(970,'TX',BLUE),(1020,'BUSY',BLUE)]:d.port(430,y,label,color)
    d.port(100,870,'GND',INK,'left');d.wire([(100,870),(65,870)],INK);d.dot(65,870,INK)
    d.rect(170,915,130,60,'#697a86','#435562',5)
    d.text(235,954,'microSD',22,'white',True,'middle')
    for x,label in [(170,'SPK1'),(350,'SPK2')]:
        d.port(x,1090,color=GREEN);d.text(x,1067,label,23,GREEN,True,'middle')
    audio_top,audio_ground=supply_capacitors(d,220,1145,spacing=170,labels_below_rail=True)
    d.wire([(570,870),(590,870),(590,1110),audio_top],AUDIO_POWER,4,dashed=True)
    d.wire([audio_ground,(65,1290)],INK,4,dashed=True)
    d.circle(235,1365,55,'#3f505d','#293741')
    d.circle(235,1365,38,'#728591','#899ba6')
    d.circle(235,1365,18,'#293741','#293741')
    d.wire([(170,1090),(80,1090),(80,1327),(193,1327)],GREEN)
    d.wire([(350,1090),(480,1090),(480,1327),(277,1327)],GREEN)
    d.port(193,1327,color=GREEN);d.port(277,1327,color=GREEN)
    d.text(350,1355,'Altoparlante',24,INK,True)
    d.text(350,1387,'Solo SPK1/SPK2',23)
    d.text(350,1419,'Nessun filo a GND',22,INK,True)
    d.wire([ports['GPIO34'],(2420,1415),(2420,750),(460,750),(460,970),(430,970)])
    d.wire([ports['GPIO39 / VN'],(2450,1470),(2450,720),(490,720),(490,1020),(430,1020)])
    d.wire([ports['GPIO4'],(1160,1030),(1160,1330),(725,1330)])
    d.resistor(650,1330,'1 kΩ')
    d.wire([(575,1330),(550,1330),(550,920),(430,920)])

    # Actual IRF9540 front-view leg order G-D-S.
    d.rect(687,805,210,76,'#b3bdc4','#87939c',5)
    d.circle(792,833,13,PAPER,'#7c8b95')
    d.rect(680,860,225,115,'#293741','#1e2a31',5)
    d.text(792,909,'IRF9540',26,'white',True,'middle')
    d.text(792,947,'P-MOSFET',22,'white',False,'middle')
    for x,label,col in [(715,'G',BLUE),(785,'D',AUDIO_POWER),(855,'S',RED)]:
        d.parts.append(f'<path d="M{x} 975V1010" stroke="#899ba6" stroke-width="11"/>')
        d.port(x,1010,color=col)
        label_x=x+24 if label=='D' else x-25 if label=='G' else x
        d.text(label_x,1042,label,24,col,True,'middle')
    d.wire([(520,760),(955,760),(955,1010),(855,1010)],RED);d.dot(520,760,RED)
    d.wire([(640,760),(640,815)],RED);d.dot(640,760,RED)
    d.resistor(640,890,'',True,BLUE,'left')
    d.text(640,795,'100 kΩ G–S',23,INK,True,'middle')
    d.wire([(640,965),(640,1010),(715,1010)])
    d.wire([(785,1010),(785,1060),(570,1060),(570,870),(430,870)],AUDIO_POWER)
    d.board(620,1090,230,180,'2N3904','H331 · NPN','#f3eee5')
    d.text(735,1228,'Pin da verificare',20,UNKNOWN,True,'middle')
    d.port(690,1090,color=BLUE);d.text(668,1083,'C',24,BLUE,True)
    d.port(850,1160,'B',BLUE)
    d.port(690,1270,color=INK);d.text(669,1260,'E',24,INK,True)
    d.wire([(715,1010),(715,1080),(690,1080),(690,1090)]);d.dot(715,1010)
    d.wire([ports['GPIO2'],(1190,975),(1190,1160),(1025,1160)])
    d.resistor(950,1160,'10 kΩ')
    d.wire([(875,1160),(850,1160)])
    d.wire([(865,1160),(865,1375)])
    d.dot(865,1160)
    d.resistor(865,1450,'100 kΩ B–GND',True,INK,'left')
    d.wire([(865,1525),(865,2150),(65,2150)],INK);d.dot(65,2150,INK)
    d.wire([(690,1270),(740,1270),(740,1300),(65,1300)],INK);d.dot(65,1300,INK)

    # HX711 and cell. The divider's junction is physically visible.
    d.board(100,1430,330,420,'HX711','')
    for y,label,color in [(1500,'VCC',RED),(1570,'SCK / CLK',BLUE),(1640,'DOUT / DT',BLUE)]:d.port(430,y,label,color)
    d.port(100,1500,'GND',INK,'left');d.wire([(100,1500),(65,1500)],INK);d.dot(65,1500,INK)
    d.wire([(430,1500),(520,1500)],RED);d.dot(520,1500,RED)
    d.wire([ports['RX2 / 16'],(1130,1085),(1130,1570),(1025,1570)])
    d.resistor(950,1570,'220 Ω')
    d.wire([(875,1570),(430,1570)])
    # Include the SCK pulldown in replicas; its presence on the original is unconfirmed.
    d.wire([(580,1570),(580,1695)],BLUE,4)
    d.dot(580,1570,BLUE)
    d.resistor(580,1770,'100 kΩ',True,INK)
    d.text(610,1815,'Da prevedere per replica',20,INK)
    d.wire([(580,1845),(580,1910),(65,1910)],INK,4)
    d.dot(65,1910,INK)
    d.wire([(430,1640),(650,1640)])
    d.resistor(725,1640,'10 kΩ')
    d.wire([(800,1640),(900,1640),(980,1640),(980,1800),(2050,1800),(2050,1360),ports['GPIO35']])
    d.dot(900,1640)
    d.wire([(900,1640),(900,1665)])
    d.resistor(900,1740,'20 kΩ',True,INK)
    d.wire([(900,1815),(900,2120),(65,2120)],INK);d.dot(65,2120,INK)
    d.text(790,1615,'Punto comune GPIO35',23,BLUE,True)
    d.rect(100,1960,330,150,'#bdc8cf','#899ba6',8)
    d.rect(202,1990,125,88,PAPER,'#8ea0ab',6)
    for x in (145,386):d.circle(x,2040,14,'#8197a4','#647e8c')
    for x,label,color in [(150,'E+',RED),(240,'E−',INK),(330,'A+',GREEN),(400,'A−','#89959f')]:
        d.port(x,1850,color=color);d.text(x,1826,label,23,color,True,'middle')
        d.wire([(x,1850),(x,1960)],color)
        d.port(x,1960,color=color)
    d.text(505,1998,'Cella di carico',27,INK,True)
    d.text(505,2036,'Rosso E+ · nero E−',24)
    d.text(505,2074,'Verde A+ · bianco A−',24)
    d.text(505,2112,'Controlla i colori della tua cella',22,MUTED)

    # OLED: five individually drawn SPI wires plus supply and ground.
    d.board(2710,1130,330,650,'OLED SSD1322','256 × 64 · VCC da ESP32 3V3')
    d.rect(2740,1225,270,55,'#20343b','#20343b',5)
    d.text(2875,1262,'MINÙ 0.000 kg',25,'#d6f7e4',True,'middle')
    for y,label in [(1310,'SCK / CLK'),(1380,'MOSI / DIN'),(1450,'CS'),(1520,'DC'),(1590,'RST / RES')]:d.port(2710,y,label,BLUE,'left')
    d.port(3040,1660,'VCC',ORANGE);d.port(3040,1730,'GND',INK)
    d.wire([(3040,1730),(3140,1730)],INK);d.dot(3140,1730,INK)
    d.wire([ports['GPIO18'],(1050,1250),(1050,1860),(2090,1860),(2090,1310),(2710,1310)])
    d.wire([ports['GPIO23'],(1080,1580),(1080,1890),(2120,1890),(2120,1380),(2710,1380)])
    d.wire([ports['GPIO25'],(2180,1195),(2180,1450),(2710,1450)])
    d.wire([ports['GPIO26'],(2210,1140),(2210,1520),(2710,1520)])
    d.wire([ports['GPIO27'],(2240,1085),(2240,1590),(2710,1590)])

    # Matrix terminal positions here are functional; all six nets are visible.
    d.board(1250,1940,760,180,'Tastiera 4 × 2','Nessun filo a VCC o GND')
    for x,label in [(1300,'R4 / 21'),(1430,'R3 / 19'),(1560,'C2 / 5'),(1690,'C1 / 17'),(1820,'R1 / 13'),(1950,'R2 / 14')]:
        d.port(x,1940,color=BLUE);d.text(x,1920,label,22,BLUE,True,'middle')
    d.text(1630,2057,'R1: SKIP / SLEEP · R2: ENTER / CLEAR',21,INK,True,'middle')
    d.text(1630,2091,'R3: WIFI / TOTAL · R4: MODE / TARE',21,INK,True,'middle')
    d.wire([ports['TX2 / 17'],(1140,1140),(1140,1720),(1690,1720),(1690,1940)])
    d.wire([ports['GPIO5'],(1110,1195),(1110,1750),(1560,1750),(1560,1940)])
    d.wire([ports['GPIO19'],(1180,1305),(1180,1840),(1430,1840),(1430,1940)])
    d.wire([ports['GPIO21'],(1210,1360),(1210,1870),(1300,1870),(1300,1940)])
    d.wire([ports['GPIO13'],(2360,920),(2360,1720),(1820,1720),(1820,1940)])
    d.wire([ports['GPIO14'],(2390,1030),(2390,1750),(1950,1750),(1950,1940)])

    # Passive buzzer output and its common ground.
    d.circle(2360,2050,70,'#293b46','#1c2b33')
    d.circle(2360,2050,17,'#14222a','#14222a')
    d.port(2290,2050,color=BLUE);d.port(2360,2120,color=INK)
    d.text(2310,2041,'+',26,'white',True)
    d.text(2400,2120,'−',26,INK,True)
    d.wire([ports['GPIO22'],(1100,1525),(1100,1830),(2240,1830),(2240,2050),(2290,2050)])
    d.wire([(2360,2120),(3140,2120)],INK);d.dot(3140,2120,INK)
    d.text(2210,2160,'Buzzer passivo · GPIO22 + GND',24,INK,True)

    # Missing physical details are explicit and are never replaced by upgrades.
    d.note(2530,1810,500,'Da rilevare sul circuito',[
        '2N3904: ordine fisico B/C/E.',
        'ESP32: valori montati da leggere.',
        '47 µF / 16 V + 104: consigliati ESP32.',
        'Caps audio tratteggiati: riferimento.',
        'Audio: fili e polarità da verificare.',
        'Fusibile F1: valore da leggere.',
        'SCK: replica 100 kΩ; montata da verificare.'
    ],280)
    d.text(70,2223,'Solo ESP32: posizione pin dalla mappa fornita. Tastiera: matrice logica misurata, ordine connettore da verificare. LED sleep invariato.',24,MUTED)
    d.save('00-wiring-complete')


if __name__ == '__main__':
    draw()
    print('Generated complete assembly map (SVG).')
