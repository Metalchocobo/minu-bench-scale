"""Generate the current scale's editable assembly plates, with explicit unknowns.

Board terminals are arranged by function, not by an unverified PCB pin order.
Resistor bodies are illustrations: read the printed values, not the drawn bands.
"""
from html import escape
from pathlib import Path

ROOT = Path(__file__).resolve().parent
W, H = 1800, 1250
INK = '#25323a'
RED = '#ca4239'
BLUE = '#226db5'
ORANGE = '#c57a12'
GREEN = '#267751'
PAPER = '#ffffff'
MUTED = '#596a76'
UNKNOWN = '#98651a'


class Plate:
    def __init__(self, number, title, subtitle):
        self.number = number
        self.wires, self.parts, self.nodes, self.panels = [], [], [], []
        self.rect(0, 0, W, H, PAPER, PAPER, 0)
        self.text(65, 47, 'MINÙ / CABLAGGIO · SCHEMI FORNITI + DOCUMENTAZIONE HX711', 19, GREEN, True)
        self.text(65, 102, f'{number:02d}   {title}', 38, INK, True)
        self.text(65, 143, subtitle, 21, MUTED)

    def text(self, x, y, s, size=24, color=INK, bold=False, anchor='start'):
        self.parts.append(f'<text x="{x}" y="{y}" font-family="Arial,Helvetica,sans-serif" font-size="{size}" font-weight="{700 if bold else 400}" fill="{color}" text-anchor="{anchor}">{escape(s)}</text>')

    def rect(self, x, y, w, h, fill='#f1f5f7', stroke='#cbd6dd', radius=16, background=False):
        layer = self.panels if background else self.parts
        layer.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{radius}" fill="{fill}" stroke="{stroke}" stroke-width="2"/>')

    def circle(self, x, y, r, fill=INK, stroke=INK, width=2):
        self.parts.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{fill}" stroke="{stroke}" stroke-width="{width}"/>')

    def wire(self, points, color=BLUE, width=6, dashed=False):
        p = ' '.join(f'{x},{y}' for x, y in points)
        dash = ' stroke-dasharray="12 10"' if dashed else ''
        self.wires.append(f'<polyline points="{p}" fill="none" stroke="white" stroke-width="{width+8}" stroke-linejoin="round" stroke-linecap="round"/>')
        self.wires.append(f'<polyline points="{p}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round" stroke-linecap="round"{dash}/>')

    def port(self, x, y, label='', color=BLUE, side='right', size=23):
        self.nodes.append(f'<circle cx="{x}" cy="{y}" r="7" fill="white" stroke="{color}" stroke-width="3"/>')
        if label:
            # Labels sit inside the illustrated module.
            self.text(x-17 if side == 'right' else x+17, y+8, label, size, color, True, 'end' if side == 'right' else 'start')

    def dot(self, x, y, color=BLUE):
        self.nodes.append(f'<circle cx="{x}" cy="{y}" r="7" fill="{color}"/>')

    def resistor(self, x, y, label, vertical=False, color=BLUE, label_side='top'):
        # The local span is 150 px; bodies have deliberately non-coded bands.
        transform = f'translate({x} {y})' + (' rotate(90)' if vertical else '')
        self.parts.append(f'<g transform="{transform}"><path d="M-75 0H-42 M42 0H75" stroke="{color}" stroke-width="6"/><rect x="-42" y="-15" width="84" height="30" rx="11" fill="#edd8ac" stroke="#a99169" stroke-width="2"/><path d="M-24 -13V13 M0 -13V13 M24 -13V13" stroke="#b69c74" stroke-width="6"/></g>')
        if vertical:
            self.text(x-28 if label_side == 'left' else x+28, y+8, label, 25, INK, True, 'end' if label_side == 'left' else 'start')
        else:
            self.text(x, y-28 if label_side == 'top' else y+48, label, 25, INK, True, 'middle')

    def board(self, x, y, w, h, title, subtitle='', color='#e8f3ee'):
        self.rect(x, y, w, h, color, '#8cab9b', 18)
        for px in (x+18, x+w-18):
            self.circle(px, y+18, 5, PAPER, '#8cab9b')
        self.text(x+w/2, y+45, title, 28, INK, True, 'middle')
        if subtitle:
            self.text(x+w/2, y+75, subtitle, 19, MUTED, False, 'middle')

    def esp(self, x, y, w, h):
        self.board(x, y, w, h, 'ESP32 DevKit', 'Usa le sigle GPIO sulla scheda', '#edf2f7')
        self.rect(x+45, y+95, w-90, 75, '#aab4bc', '#87939c', 5)
        self.text(x+w/2, y+140, 'ESP32', 25, INK, True, 'middle')

    def note(self, x, y, w, title, lines, h=None):
        h = h or (61+len(lines)*31)
        self.rect(x, y, w, h, '#fff7e8', '#dfc79a')
        self.text(x+22, y+36, title, 24, UNKNOWN, True)
        for i, line in enumerate(lines):
            self.text(x+22, y+70+i*31, line, 21)

    def footer(self, extra='I terminali sono disposti per leggibilità: segui le sigle, non la posizione nel disegno.'):
        for x, c, label in [(65, RED, 'Positivo 5 V'), (380, INK, 'GND comune'), (715, ORANGE, '3,3 V'), (950, BLUE, 'Segnali')]:
            self.wire([(x, 1177), (x+52, 1177)], c)
            self.text(x+68, 1185, label, 21)
        self.text(1235, 1185, '● = unione dei fili', 21)
        self.text(65, 1224, extra, 19, MUTED)

    def save(self, stem):
        # Background first, then wires; components/labels mask wire crossings.
        background, foreground = self.parts[:1], self.parts[1:]
        svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" role="img">',
               f'<title>Minù: tavola {self.number} del cablaggio corrente</title>',
               '<desc>Illustrazione di montaggio. I valori ignoti sono marcati da confermare. I terminali dei moduli seguono le sigle, non una posizione fisica universale.</desc>']
        (ROOT/f'{stem}.svg').write_text('\n'.join(svg+background+self.panels+self.wires+foreground+self.nodes+['</svg>']), encoding='utf-8')


def electrolytic(d, x, y, label, top=810, bottom=1030):
    # Two physical leads leave the can at its bottom; neither is in series.
    d.wire([(x-95, top), (x-95, y+91), (x-27, y+91), (x-27, y+75)], RED)
    d.wire([(x+27, y+75), (x+27, bottom)], INK)
    d.rect(x-58, y-55, 116, 130, '#354b5a', '#263b48', 10)
    d.rect(x+19, y-52, 22, 126, '#ced9de', '#ced9de', 0)
    d.parts.append(f'<ellipse cx="{x}" cy="{y-55}" rx="58" ry="15" fill="#c2ccd1" stroke="#71838c" stroke-width="2"/>')
    d.text(x+30, y+25, '−', 29, INK, True, 'middle')
    d.text(x-12, y+95, '+', 24, RED, True, 'middle')
    d.text(x, top-20, label, 25, INK, True, 'middle')
    d.dot(x-95, top, RED)
    d.dot(x+27, bottom, INK)


def ceramic(d, x, y, label, top=810, bottom=1030):
    d.wire([(x-65, top), (x-65, y+70), (x-18, y+70), (x-18, y+35)], RED)
    d.wire([(x+18, y+35), (x+18, bottom)], INK)
    d.circle(x, y, 39, '#c88839', '#9d672b')
    d.text(x, y+8, '104', 23, '#fff7e8', True, 'middle')
    d.text(x, top-20, label, 24, INK, True, 'middle')
    d.dot(x-65, top, RED)
    d.dot(x+18, bottom, INK)


def supply_capacitors(d, x, y, spacing=200, labels_below_rail=False,
                      capacity='470 µF', voltage='16 V', dashed=True):
    """Draw a parallel supply pair; dashed wiring denotes unverified terminals."""
    left, right = x-120, x+spacing+55
    top, bottom = y-35, y+145
    d.wire([(left,top),(right,top)],RED,4,dashed=dashed)
    d.wire([(left,bottom),(right,bottom)],INK,4,dashed=dashed)
    d.wire([(x-75,top),(x-75,y+128),(x-28,y+128),(x-28,y+110)],RED,4,dashed=dashed)
    d.wire([(x+28,y+110),(x+28,bottom)],INK,4,dashed=dashed)
    d.rect(x-50,y,100,110,'#354b5a','#263b48',8)
    d.rect(x+20,y+4,16,102,'#ced9de','#ced9de',0)
    d.parts.append(f'<ellipse cx="{x}" cy="{y}" rx="50" ry="12" fill="#bdc9d0" stroke="#728895" stroke-width="2"/>')
    d.text(x-12,y+42,capacity,17,'white',True,'middle')
    d.text(x-8,y+73,voltage,21,'white',True,'middle')
    d.text(x+28,y+101,'−',22,INK,True,'middle')
    d.text(x-28,y+126,'+',21,RED,True,'middle')
    label_y=y-5 if labels_below_rail else y-55
    d.text(x,label_y,'Elettrolitico',22,INK,True,'middle')
    cx=x+spacing
    d.wire([(cx-55,top),(cx-55,y+128),(cx-18,y+128),(cx-18,y+87)],RED,4,dashed=dashed)
    d.wire([(cx+18,y+87),(cx+18,bottom)],INK,4,dashed=dashed)
    d.circle(cx,y+60,32,'#c88839','#9d672b')
    d.text(cx,y+68,'104',22,'white',True,'middle')
    d.text(cx,label_y,'Ceramico',22,INK,True,'middle')
    if not dashed:
        for px,py,color in [(x-75,top,RED),(x+28,bottom,INK),(cx-55,top,RED),(cx+18,bottom,INK)]:
            d.dot(px,py,color)
    return (left,top),(right,bottom)


def overview():
    d = Plate(1, 'Tutta la bilancia, a colpo d’occhio', 'Le tavole 2–6 mostrano ogni filo e resistenza; i valori ancora ignoti restano da confermare.')
    # The power path is visible above the signal hub.
    for x, w, title, sub in [(65,280,'NASTIMA','6,4 V · 6 Ah · USB-C'), (440,250,'INA219','Positivo VIN+ → VIN−'), (795,190,'S1','Interruttore'), (1090,270,'Mini360','OUT+ regolato a 5,11 V')]:
        d.board(x,220,w,160,title,sub)
    d.text(390,278,'F1',23,UNKNOWN,True,'middle')
    d.text(1450,277,'Linea 5 V',25,RED,True)
    d.text(1450,312,'VIN ESP32 · HX711',21)
    d.text(1450,344,'Audio attraverso IRF9540',21)
    d.wire([(345,330),(440,330)],RED)
    d.wire([(690,330),(795,330)],RED)
    d.wire([(985,330),(1090,330)],RED)
    d.wire([(1360,330),(1420,330),(1420,416),(880,416),(880,505)],RED)
    for x in (345,440,690,795,985,1090,1360): d.port(x,330,color=RED)
    d.text(65,195,'Ricarica USB-C direttamente sul pacco NASTIMA. GND comune a ESP32 e moduli.',22)
    d.esp(710,505,365,390)
    d.text(892,730,'GPIO = sigla del pin',23,INK,True,'middle')
    d.text(892,771,'Nessun 5 V sui GPIO',22,INK,False,'middle')
    d.text(892,823,'OLED + INA219 da 3V3',22,ORANGE,True,'middle')
    d.board(65,495,465,285,'Cella + HX711','Tavola 3')
    for i,line in enumerate(['E+ / E− / A+ / A− → cella', 'GPIO16 → 220 Ω → SCK', 'DOUT → 10 kΩ → GPIO35', 'GPIO35 → 20 kΩ → GND', 'VCC 5 V · GND comune']): d.text(92,612+i*32,line,23)
    d.wire([(530,660),(620,660),(620,620),(710,620)])
    d.port(530,660);d.port(710,620)
    d.board(1325,495,410,265,'OLED SSD1322','Tavola 6')
    for i,line in enumerate(['18 → SCK · 23 → DIN/MOSI','25 → CS · 26 → DC · 27 → RST','VCC da ESP32 3V3 · GND comune','Collegamenti dallo schema fornito']):d.text(1350,612+i*32,line,21)
    d.wire([(1075,620),(1190,620),(1190,645),(1325,645)])
    d.port(1075,620);d.port(1325,645)
    d.board(1325,805,410,305,'DFPlayer + altoparlante','Tavola 4 · IRF9540 + 2N3904')
    for i,line in enumerate(['GPIO4 → 1 kΩ → RX','TX → GPIO34 · BUSY → GPIO39','GPIO2 → 10 kΩ → base NPN','100 kΩ G–S · 100 kΩ B–GND','VCC dal MOSFET · SPK1/SPK2','Audio: 470 µF / 16 V + 104']):d.text(1350,912+i*31,line,21)
    d.wire([(1075,790),(1200,790),(1200,980),(1325,980)])
    d.port(1075,790);d.port(1325,980)
    d.board(65,830,465,280,'LED rosso + buzzer','Tavola 5')
    for i,line in enumerate(['GPIO15 → 1 kΩ → LED +','LED − → GND','GPIO22 → segnale buzzer','Buzzer − → GND']):d.text(92,947+i*34,line,23)
    d.wire([(530,970),(600,970),(600,790),(710,790)])
    d.port(530,970);d.port(710,790)
    d.board(635,960,570,150,'Tastiera 4 × 2','Tavola 5 · soltanto 6 fili, senza VCC / GND')
    d.text(660,1076,'R1/2/3/4 → 17/5/13/14 · C1/2 → 19/21',22)
    d.wire([(890,895),(890,960)])
    d.port(890,895);d.port(890,960)
    d.text(635,1147,'PCB custom e ESP32: due coppie distinte; dettagli nella tavola 2.',20,MUTED)
    d.footer('Questa panoramica raggruppa i fili. Per saldare usa le tavole di dettaglio e le sigle reali sui moduli.')
    d.save('01-wiring-overview')


def power():
    d=Plate(2,'Batteria, alimentazione e condensatori','Mini360: 5 V a VIN ESP32, HX711 e audio. VCC OLED e INA219 provengono dall’ESP32 a 3,3 V.')
    d.note(65,180,1670,'Ricarica del pacco', ['USB esterna → prolunga USB-C di scocca → USB-C NASTIMA. Il caricatore/BMS è interno al pacco.'],108)
    d.board(65,330,235,220,'NASTIMA','6,4 V / 6 Ah','#dce7ed')
    d.port(300,435,'+',RED);d.port(300,505,'−',INK)
    d.wire([(300,435),(365,435)],RED)
    d.rect(365,413,140,44,'#e9d8bc','#b4a17e',8)
    d.text(435,444,'F1',26,INK,True,'middle')
    d.text(435,388,'Fusibile vicino al +',22,INK,False,'middle')
    d.text(435,490,'Valore da confermare',21,UNKNOWN,True,'middle')
    d.wire([(505,435),(610,435)],RED)
    d.board(610,330,300,220,'INA219','Il positivo attraversa lo shunt')
    d.port(610,435,'VIN+',RED,'left');d.port(910,435,'VIN−',RED)
    d.text(760,505,'VIN− NON è GND',24,RED,True,'middle')
    d.wire([(910,435),(1020,435)],RED)
    d.rect(1020,390,170,95,'#f1f3f5','#9eabb5',10)
    d.rect(1090,410,40,43,'#35434c','#35434c',5)
    d.text(1105,365,'S1 · ON/OFF',24,INK,True,'middle')
    d.wire([(1190,435),(1325,435)],RED)
    d.board(1325,330,360,220,'Mini360','Morsetti per sigla, non vista PCB')
    d.port(1325,435,'IN+',RED,'left');d.port(1325,505,'IN−',INK,'left')
    d.port(1685,435,'OUT+',RED);d.port(1685,505,'OUT−',INK)
    d.text(1505,590,'Montato: 5,11 V',27,RED,True,'middle')
    d.wire([(300,505),(330,505),(330,650),(1740,650)],INK)
    d.wire([(1325,505),(1270,505),(1270,650)],INK)
    d.wire([(1685,505),(1740,505),(1740,650)],INK)
    d.dot(1270,650,INK);d.dot(1740,650,INK)
    d.text(355,638,'GND comune → ESP32, HX711, OLED, INA219, DFPlayer, LED e buzzer',24,INK,True)
    d.wire([(1685,435),(1770,435),(1770,600)],RED)
    d.port(1770,600,color=RED)
    d.text(65,690,'5 V → VIN ESP32 / HX711 / audio · ESP32 3V3 → OLED / VCC INA219',23,RED,True)
    for x,w,title,plus,minus,el,voltage,spacing in [(65,400,'Mini360 IN · dopo S1','IN+','IN−','220 µF','16 V',170),(495,400,'Mini360 OUT','OUT+','OUT−','470 µF','10 V',170),(925,400,'PCB custom · 5 V/GND','5 V','GND','47 µF','16 V',170),(1355,400,'Presso ESP32 · VIN/GND','VIN','GND','47 µF','16 V',170)]:
        d.rect(x,710,w,405,'#f5f8fa','#cbd6dd',background=True)
        d.text(x+25,749,title,24,INK,True)
        cap_x=x+150 if x==495 else x+135
        top_node,bottom_node=supply_capacitors(d,cap_x,855,spacing=spacing,capacity=el,voltage=voltage,dashed=False)
        d.port(*top_node,color=RED);d.port(top_node[0],bottom_node[1],color=INK)
        d.text(x+25,808,plus,20,RED,True);d.text(x+25,1031,minus,20,INK,True)
        d.text(x+25,1062,'Parallelo · 104 = 100 nF.',20)
        if x==1355:
            d.text(x+25,1093,'Valori per replica; montati da leggere.',18,UNKNOWN,True)
        elif x==925:
            d.text(x+25,1093,'Coppia aggiuntiva confermata.',20,INK,True)
        else:
            d.text(x+25,1093,'Vicino ai pad, con fili corti.',20)
    d.footer('Banda − del condensatore al negativo; ceramico senza polarità. Retro Mini360: vedi tavola pad dedicata.')
    d.save('02-power')


def hx711():
    d=Plate(3,'HX711 e cella di carico','HX711 alimentato dalla linea 5 V; il partitore è sul solo filo DOUT diretto a GPIO35.')
    d.esp(65,230,375,660)
    for y,label,c in [(400,'GPIO16 / RX2',BLUE),(580,'GPIO35',BLUE),(750,'Linea 5 V',RED),(820,'GND',INK)]:d.port(440,y,label,c)
    d.board(1070,230,305,660,'HX711','Breakout HX711')
    d.rect(1138,455,130,70,'#304b3e','#304b3e',5)
    d.text(1203,500,'HX711',22,'white',True,'middle')
    for y,label,c in [(400,'SCK / CLK',BLUE),(580,'DOUT / DT',BLUE),(750,'VCC',RED),(820,'GND',INK)]:d.port(1070,y,label,c,'left')
    d.wire([(440,400),(625,400)])
    d.resistor(700,400,'220 Ω')
    d.wire([(775,400),(1070,400)])
    # Include this pulldown in replicas, on the HX711 side of the series resistor.
    d.wire([(970,400),(970,410)],BLUE,4)
    d.dot(970,400,BLUE)
    d.resistor(970,485,'100 kΩ',True,INK,'left')
    d.wire([(970,560),(970,820)],INK,4)
    d.dot(970,820,INK)
    d.wire([(1070,580),(925,580)])
    d.resistor(850,580,'10 kΩ')
    d.wire([(775,580),(440,580)])
    d.wire([(650,580),(650,590)])
    d.resistor(650,665,'20 kΩ',True,INK)
    d.wire([(650,740),(650,820)],INK)
    d.wire([(440,750),(1070,750)],RED)
    d.wire([(440,820),(1070,820)],INK)
    d.dot(650,580);d.dot(650,820,INK)
    d.text(485,510,'Punto comune GPIO35',23,INK,True)
    d.text(770,873,'20 kΩ scende a GND; non va in fila su DOUT.',22,INK,True,'middle')
    # An aluminium load cell illustration; colours are a documented convention.
    d.rect(1520,260,210,420,'#bdc8cf','#899ba6',8)
    d.rect(1558,363,134,170,'#ffffff','#8ea0ab',7)
    for y in (300,638):
        for x in (1565,1685):d.circle(x,y,13,'#8399a6','#647e8c')
    d.text(1625,726,'CELLA DI CARICO',23,INK,True,'middle')
    for y,label,c,desc in [(300,'E+',RED,'Rosso'),(370,'E−',INK,'Nero'),(440,'A+',GREEN,'Verde'),(510,'A−','#8a96a0','Bianco')]:
        d.port(1375,y,label,c)
        d.wire([(1375,y),(1520,y)],c)
        d.text(1445,y-18,desc,21,INK,False,'middle')
        d.port(1520,y,color=c,side='left')
    d.note(65,950,1670,'Prima di replicare',[
        'I colori della cella sono la convenzione documentata: controlla E+ / E− / A+ / A− della cella che usi.',
        'Replica: prevedere 100 kΩ fra SCK e GND, dopo la 220 Ω lato HX711, vicino al modulo.',
        'VCC a 5 V ed E+/E− verso la cella non significano 5 V sui GPIO dell’ESP32.'
    ],167)
    d.footer()
    d.save('03-hx711')


def audio():
    d=Plate(4,'DFPlayer, MOSFET, transistor e altoparlante','Resistenze come nello schema fornito: 1 kΩ sulla UART, 10 kΩ alla base e due resistenze da 100 kΩ.')
    d.wire([(65,190),(690,190),(690,420),(610,420)],RED)
    d.text(65,220,'Linea 5 V dal Mini360',23,RED,True)
    # IRF9540: front face and known G-D-S leg order.
    d.rect(423,245,235,86,'#b3bdc4','#87939c',7)
    d.circle(541,270,14,PAPER,'#7c8b95')
    d.rect(415,310,250,80,'#293741','#1e2a31',5)
    d.text(540,347,'IRF9540',27,'white',True,'middle')
    d.text(540,378,'P-channel',21,'white',False,'middle')
    for x,label,c in [(470,'G',BLUE),(540,'D',RED),(610,'S',RED)]:
        d.parts.append(f'<path d="M{x} 390V420" stroke="#899ba6" stroke-width="12"/>')
        d.port(x,420,color=c)
        d.text(x+28 if label == 'D' else x,452,label,24,c,True,'middle')
    d.text(695,291,'Fronte con scritta leggibile',23,INK,True)
    d.text(695,329,'Piedini verso il basso: G · D · S',23)
    d.text(695,367,'Linguetta metallica = D',23)
    d.wire([(365,190),(365,240)],RED)
    d.resistor(365,315,'100 kΩ G–S',True,BLUE,'left')
    d.wire([(365,390),(365,420),(400,420)])
    d.dot(365,190,RED)
    d.wire([(470,420),(400,420),(400,640),(810,640)])
    d.dot(470,420)
    d.dot(400,420)
    d.wire([(540,420),(540,590),(1170,590),(1170,710),(1270,710)],RED)
    d.text(865,581,'D → VCC DFPlayer',23,RED,True,'middle')
    # Functional connection boundary deliberately avoids a guessed NPN pin order.
    d.board(720,640,285,175,'2N3904','H331 · NPN montato','#f3eee5')
    d.rect(830,731,75,45,'#293741','#293741',9)
    d.text(867,760,'NPN',20,'white',True,'middle')
    d.port(810,640,'',BLUE)
    d.text(810,623,'C · collettore',22,BLUE,True,'middle')
    d.port(720,730,'B · base',BLUE,'left',21)
    d.port(810,815,'',INK)
    d.text(842,839,'E · emettitore',22,INK,True)
    d.board(65,635,255,175,'ESP32','Comando audio')
    d.port(320,730,'GPIO2',BLUE)
    d.wire([(320,730),(425,730)])
    d.resistor(500,730,'10 kΩ')
    d.wire([(575,730),(720,730)])
    d.dot(650,730)
    d.wire([(650,730),(650,740)])
    d.resistor(650,815,'100 kΩ B–GND',True,INK,'left')
    d.wire([(650,890),(650,1060)],INK)
    d.wire([(810,815),(810,1060)],INK)
    d.rect(1065,185,670,355,'#fff7e8','#dfc79a',background=True)
    d.text(1087,222,'Condensatori audio presenti',24,UNKNOWN,True)
    audio_top,audio_ground=supply_capacitors(d,1210,315,spacing=230)
    d.wire([(1170,590),(1170,555),(1050,555),(1050,280),audio_top],RED,4,dashed=True)
    d.wire([audio_ground,(1770,460),(1770,1060),(1720,1060)],INK,4,dashed=True)
    d.text(1087,497,'Tratteggi: riferimento VCC DFPlayer / GND.',21,UNKNOWN,True)
    d.text(1087,529,'470 µF / 16 V + 100 nF (104), confermati.',21)
    d.board(1270,670,285,370,'','')
    d.text(1413,650,'DFPlayer Mini',27,INK,True,'middle')
    d.rect(1348,773,130,75,'#69757e','#43515b',4)
    d.text(1413,818,'microSD',22,'white',True,'middle')
    for y,label,c in [(710,'VCC',RED),(875,'RX',BLUE),(930,'TX',BLUE),(985,'BUSY',BLUE)]:d.port(1270,y,label,c,'left')
    d.port(1410,1040,color=INK)
    d.text(1434,1030,'GND',22,INK,True)
    d.board(65,840,255,200,'','')
    d.text(190,828,'ESP32 · UART',24,INK,True,'middle')
    for y,label in [(875,'GPIO4'),(930,'GPIO34'),(985,'GPIO39 / VN')]:d.port(320,y,label,BLUE,size=22)
    d.wire([(320,875),(400,875)])
    d.resistor(475,875,'1 kΩ')
    d.wire([(550,875),(1100,875),(1100,875),(1270,875)])
    d.wire([(320,930),(1270,930)])
    d.wire([(320,985),(1270,985)])
    d.wire([(65,1060),(1720,1060)],INK)
    d.wire([(1410,1040),(1410,1060)],INK)
    for x in (650,810,1410):d.dot(x,1060,INK)
    d.text(1600,1100,'GND comune',22,INK,True)
    # Two separate speaker wires: neither is ground.
    for y,label in [(750,'SPK1'),(805,'SPK2')]:d.port(1555,y,label,GREEN)
    d.wire([(1555,750),(1590,750),(1590,760),(1630,760)],GREEN)
    d.wire([(1555,805),(1608,805),(1608,815),(1630,815)],GREEN)
    d.circle(1690,788,69,'#3f505d','#293741')
    d.circle(1690,788,48,'#697d8a','#899ba6')
    d.circle(1690,788,22,'#293741','#293741')
    d.port(1630,760,color=GREEN);d.port(1630,815,color=GREEN)
    d.text(1690,888,'Altoparlante',21,INK,True,'middle')
    d.text(1690,919,'SPK1 ↔ SPK2',21,INK,False,'middle')
    d.text(1660,950,'Nessun filo a GND',19,INK,True,'middle')
    d.text(65,1122,'GPIO2 HIGH = audio acceso. BUSY è diretto nello schema; la pull-up opzionale non è mostrata come montata.',22)
    d.footer('2N3904: B/C/E funzionali, ordine da verificare. Caps audio: fili e polarità di riferimento da riscontrare.')
    d.save('04-audio')


def controls():
    d=Plate(5,'Tastiera, LED rosso e buzzer','Il cablaggio e il comportamento sleep restano quelli attuali. Resistenza LED: 1 kΩ nello schema fornito.')
    d.esp(65,220,325,890)
    d.board(1270,220,465,555,'Tastiera 4 × 2','6 fili · nessun VCC / GND')
    rows=[(355,'GPIO17','Filo 1 · R1'),(425,'GPIO5','Filo 2 · R2'),(495,'GPIO13','Filo 3 · R3'),(565,'GPIO14','Filo 4 · R4'),(635,'GPIO19','Filo 5 · C1'),(705,'GPIO21','Filo 6 · C2')]
    for y,pin,label in rows:
        d.port(390,y,pin);d.port(1270,y,label,BLUE,'left',21)
        d.wire([(390,y),(1270,y)])
    for row,labels in enumerate([('SKIP','ENTER'),('WIFI','TOTAL'),('SLEEP','MODE'),('CLEAR','TARE')]):
        for col,label in enumerate(labels):
            d.rect(1482+col*113,330+row*92,98,70,'#f7f9fa','#9baaaf',8)
            d.text(1531+col*113,373+row*92,label,21,INK,True,'middle')
    d.text(600,766,'R1–R4 e C1–C2: verificare l’ordine del connettore della tastiera.',22)
    d.port(390,875,'GPIO22')
    d.wire([(390,875),(1325,875)])
    d.circle(1400,875,72,'#2b3b45','#1c2b33')
    d.circle(1400,875,18,'#152129','#152129')
    d.port(1325,875,color=BLUE)
    d.text(1350,862,'+',26,'white',True)
    d.port(1400,948,color=INK)
    d.text(1420,941,'−',26,INK,True)
    d.text(1530,869,'Buzzer passivo',26,INK,True)
    d.text(1530,905,'Segnale → GPIO22',22)
    d.text(1530,941,'Negativo → GND',22)
    d.port(390,1010,'GPIO15')
    d.wire([(390,1010),(565,1010)])
    d.resistor(640,1010,'1 kΩ LED')
    d.wire([(715,1010),(1130,1010)])
    # Red LED body and two physical leads.
    d.parts.append('<path d="M1110 991V957 A47 47 0 0 1 1204 957V991Z" fill="#db534b" stroke="#a83230" stroke-width="3"/>')
    d.rect(1102,985,110,13,'#bd3835','#bd3835',4)
    d.port(1130,1010,'',BLUE);d.port(1190,1010,'',INK)
    d.parts.append('<path d="M1130 997V1010 M1190 997V1010" stroke="#8a979f" stroke-width="6"/>')
    d.text(1119,1050,'Anodo +',22,BLUE,True,'end')
    d.text(1215,1001,'LED rosso',25,INK,True)
    d.text(1215,1040,'Catodo − → GND',22)
    d.wire([(1190,1010),(1190,1090)],INK)
    d.wire([(1400,948),(1488,948),(1488,1090)],INK)
    d.port(390,1090,'GND',INK)
    d.wire([(390,1090),(1735,1090)],INK)
    d.dot(1190,1090,INK);d.dot(1488,1090,INK)
    d.text(1540,1074,'GND comune',22,INK,True)
    d.text(65,1140,'LED: catodo = lato piatto del corpo; gambetta corta se non tagliata. Conservare la resistenza montata.',22)
    d.footer()
    d.save('05-controls')


def display_ina():
    d=Plate(6,'Display OLED e segnali INA219','OLED e INA219 ricevono VCC dall’ESP32 3V3, come nello schema fornito.')
    d.esp(65,220,345,905)
    d.board(1270,220,465,600,'OLED SSD1322 · 256 × 64','Sigle equivalenti secondo il modulo')
    d.rect(1340,313,325,50,'#20343b','#20343b',5)
    d.text(1502,346,'MINÙ      0.000 kg',23,'#d6f7e4',True,'middle')
    for y,src,dst,c in [(380,'GPIO18','SCK / CLK',BLUE),(450,'GPIO23','MOSI / DIN',BLUE),(520,'GPIO25','CS',BLUE),(590,'GPIO26','DC',BLUE),(660,'GPIO27','RST / RES',BLUE),(730,'3V3','VCC',ORANGE),(800,'GND','GND',INK)]:
        d.port(410,y,src,c)
        d.port(1270,y,dst,c,'left',22)
        d.wire([(410,y),(1270,y)],c)
    d.board(1270,870,365,255,'INA219','')
    for y,src,dst,c in [(945,'GPIO32','SDA',BLUE),(1005,'GPIO33','SCL',BLUE),(1065,'3V3','VCC',ORANGE)]:
        d.port(410,y,src,c)
        d.port(1270,y,dst,c,'left')
        d.wire([(410,y),(1270,y)],c)
    d.port(1450,1125,color=INK)
    d.text(1470,1110,'GND',22,INK,True)
    d.wire([(1170,800),(1170,1138),(1450,1138),(1450,1125)],INK)
    d.dot(1170,800,INK)
    d.port(1635,945,'VIN+',RED)
    d.port(1635,1040,'VIN−',RED)
    d.text(1665,967,'Potenza',21,RED,True)
    d.text(1665,1000,'tavola 2',21,RED)
    d.text(65,1152,'VCC OLED e INA219 a 3V3. Le posizioni dei loro terminali nel disegno sono funzionali: seguire le sigle.',22)
    d.footer('INA219 VIN+/VIN− appartengono al positivo batteria. VCC / SDA / SCL / GND sono collegamenti separati.')
    d.save('06-display-ina')


if __name__ == '__main__':
    for draw in (overview, power, hx711, audio, controls, display_ina):
        draw()
    print('Generated 6 assembly plates (SVG).')
