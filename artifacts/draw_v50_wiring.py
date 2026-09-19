from pathlib import Path
from html import escape

ROOT = Path(__file__).resolve().parent
RED, BLACK, PURPLE, BLUE, BROWN, ORANGE = '#c93832', '#26333d', '#7752b3', '#216fc0', '#83502c', '#c47713'

class Drawing:
    def __init__(self, title, subtitle, height=1530):
        self.height = height
        self.items = [f'<svg xmlns="http://www.w3.org/2000/svg" width="2100" height="{height}" viewBox="0 0 2100 {height}"><title>{escape(title)}</title><rect width="2100" height="{height}" fill="#f7f9fc"/>']
        self.nodes = []
        self.text(50, 60, title, 36, bold=True)
        self.text(50, 104, subtitle, 23)
    def text(self, x, y, value, size=22, color=BLACK, bold=False, anchor='start'):
        self.items.append(f'<text x="{x}" y="{y}" font-family="Arial,Helvetica,sans-serif" font-size="{size}" font-weight="{700 if bold else 400}" fill="{color}" text-anchor="{anchor}">{escape(value)}</text>')
    def box(self, x, y, w, h, fill='#fff', stroke='#b8c7d5', dash=False):
        self.items.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="12" fill="{fill}" stroke="{stroke}" stroke-width="2"'+(' stroke-dasharray="9 6"' if dash else '')+'/>')
    def wire(self, pts, color=RED, width=5, dash=False):
        p = ' '.join(f'{x},{y}' for x,y in pts)
        self.items.append(f'<polyline points="{p}" fill="none" stroke="#f7f9fc" stroke-width="{width+7}" stroke-linejoin="round" stroke-linecap="round"/>')
        self.items.append(f'<polyline points="{p}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round" stroke-linecap="round"'+(' stroke-dasharray="8 7"' if dash else '')+'/>')
    def dot(self,x,y,color=RED):
        self.nodes.append(f'<circle cx="{x}" cy="{y}" r="7" fill="{color}"/>')
    def port(self,x,y,color=RED):
        self.nodes.append(f'<circle cx="{x}" cy="{y}" r="6" fill="white" stroke="{color}" stroke-width="3"/>')
    def fuse(self,x,y,label,value):
        self.box(x,y-24,120,48,'#fff3df',ORANGE)
        self.text(x+60,y+8,label,23,bold=True,anchor='middle')
        self.text(x+60,y+62 if label=='F1' else y-38,value,22,anchor='middle')
    def cap(self,x,y,label,value,polar=True,label_x=None):
        self.wire([(x-22,y),(x+22,y)],BLACK,4)
        self.wire([(x-22,y+18),(x+22,y+18)],BLACK,4)
        if polar:
            self.text(x+28,y-8,'+',24,RED,bold=True)
            self.text(x+28,y+39,'−',24,BLACK,bold=True)
        tx = x+60 if label_x is None else label_x
        self.text(tx,y-3,label,23,bold=True)
        self.text(tx,y+28,value,20)
    def resistor(self,x,y,w,label,value,color=PURPLE):
        self.box(x,y-18,w,36,'#fff',color)
        self.text(x+w/2,y-40,label+' · '+value,22,anchor='middle',bold=True)
    def footer(self,a,b):
        self.text(50,self.height-65,a,21)
        self.text(50,self.height-30,b,20)
    def save(self,name):
        (ROOT/name).write_text('\n'.join(self.items+self.nodes+['</svg>']),encoding='utf-8')

d=Drawing('1 / 2 · Alimentazione completa: tutti i positivi e i negativi',
          'Voltaic V50 + Pololu 3728 · Schema elettrico di collegamento, non posizione fisica dei piedini.')

# Mains stays outside the scale, inside a finished plug-in supply.
d.box(50,145,760,225,'#fff4e6','#d2a574')
d.text(75,178,'ESTERNO ALLA BILANCIA · 230 V solo alla spina',22,bold=True)
d.box(80,200,190,140)
d.text(105,230,'Presa 230 V~',22,bold=True)
d.text(105,270,'L · fase',23,BROWN)
d.text(105,320,'N · neutro',23,BLUE)
d.box(400,200,360,140)
d.text(425,237,'ALIMENTATORE COMPLETO',22,bold=True)
d.text(425,270,'Raspberry Pi · 5,1 V / 3 A',22)
d.text(425,307,'Spina e cavo USB-C integrati',20)
d.wire([(270,260),(400,260)],BROWN)
d.wire([(270,310),(400,310)],BLUE)
d.text(835,186,'Cavo USB-C: coppia di alimentazione',21)
d.text(775,220,'+',24,RED,bold=True)
d.text(775,286,'GND',21,BLACK,bold=True)

# Main power modules.
d.box(1050,410,400,420,'#edf4ff','#6893bc')
d.text(1090,565,'POLOLU 3728',29,bold=True)
d.text(1090,600,'TPS2116 · usb09a',24)
d.text(1090,635,'Selezione automatica',23)
d.text(1090,765,'R1 / R2 / MODE: tavola 2',21)
d.text(1070,458,'VIN1 / USB-C +',22,RED,bold=True)
d.text(1070,512,'USB-C GND',21)
d.text(1070,703,'VIN2',22,RED,bold=True)
d.text(1340,445,'VOUT',22,RED,bold=True)
d.text(1080,805,'GND',22,bold=True)
for x,y,c in [(1050,450,RED),(1050,505,BLACK),(1050,695,RED),(1050,790,BLACK),(1450,450,RED)]:d.port(x,y,c)
d.text(1100,735,'Tutti i GND della scheda sono comuni',17)

d.box(100,500,570,370,'#edf8f1','#65a182')
d.text(130,540,'VOLTAIC V50 · pacco completo',28,bold=True)
d.text(130,578,'+  USB-C LATERALE · ingresso ricarica',22,RED,bold=True)
d.text(130,633,'−  GND dello stesso cavo',22)
d.text(355,702,'USB-A · uscita +5 V  +',22,RED,bold=True)
d.text(380,787,'USB-A · negativo / GND  −',20)
d.text(130,835,'La USB-C SUPERIORE è usata solo per la misura: tavola 2.',18)
for x,y,c in [(100,570,RED),(100,625,BLACK),(670,695,RED),(670,780,BLACK)]:d.port(x,y,c)

# Source wiring and charge branch.
d.wire([(760,225),(940,225),(940,450),(1050,450)])
d.wire([(940,450),(940,390),(670,390)])
d.fuse(550,390,'F1','T2,5 A')
d.wire([(550,390),(50,390),(50,570),(100,570)])
d.dot(940,450)
d.dot(940,390)
d.text(80,425,'Ramo di ricarica preso da VIN1, prima della selezione',21)
d.wire([(760,305),(850,305),(850,505),(1050,505)],BLACK)
d.text(1660,385,'Interruttore sul positivo',22,bold=True)

# Battery branch, output fuse and main switch.
d.wire([(670,695),(740,695)])
d.fuse(740,695,'F3','T1,25 A')
d.wire([(860,695),(1050,695)])
d.wire([(1450,450),(1650,450)])
d.fuse(1650,450,'F2','T1,25 A')
d.wire([(1770,450),(1870,450)])
d.port(1870,450);d.port(1940,450)
d.wire([(1870,450),(1925,420)])
d.wire([(1940,450),(1990,450),(1990,590),(1850,590),(1850,620)])
d.text(1905,492,'S1 · ON/OFF',22,anchor='middle',bold=True)

# INA219: VIN- is still the positive conductor.
d.box(1660,620,350,230)
d.text(1685,660,'INA219',28,bold=True)
d.text(1685,695,'VIN+',22,RED,bold=True)
d.text(1685,820,'VIN−',22,RED,bold=True)
d.wire([(1850,620),(1850,695)])
d.box(1825,695,50,50,'#fff2ed',RED)
d.text(1890,726,'shunt',20)
d.wire([(1850,745),(1850,850),(1850,1120),(350,1120)])
d.port(1850,620);d.port(1850,850)
d.text(1685,756,'VCC/SDA/SCL:',19)
d.text(1685,783,'tavola 2',19)
d.text(1710,841,'GND separato',19)
d.port(1660,800,BLACK)
d.text(1100,885,'VIN− INA219 è ancora +5 V.',22,RED,bold=True)
d.text(1100,916,'Non collegare VIN− a GND.',22,RED,bold=True)

# Capacitors, all across supply and common ground.
d.wire([(990,450),(990,550)])
d.dot(990,450)
d.cap(990,550,'C1','100 µF / 10 V',label_x=820)
d.wire([(990,568),(990,975)],BLACK)
d.wire([(920,695),(920,865)])
d.dot(920,695)
d.cap(920,865,'C2','100 µF / 10 V',label_x=735)
d.wire([(920,883),(920,975)],BLACK)
d.wire([(1550,450),(1550,550)])
d.dot(1550,450)
d.cap(1550,550,'C3','470 µF / 10 V',label_x=1605)
d.wire([(1550,568),(1550,975)],BLACK)

# Every DC negative is explicitly drawn. Crossings without dots are not junctions.
d.wire([(100,625),(75,625),(75,1380)],BLACK)
d.wire([(670,780),(705,780),(705,975)],BLACK)
d.wire([(1050,790),(1030,790),(1030,975)],BLACK)
d.wire([(1660,800),(1605,800),(1605,975)],BLACK)
d.wire([(75,975),(1740,975)],BLACK,6)
d.text(180,1020,'GND / 0 V · negativo comune di tutti i collegamenti a bassa tensione',25,bold=True)
d.text(180,1054,'Questa linea NON è collegata al neutro N della rete elettrica.',23,BLACK,bold=True)
for x in [75,705,920,990,1030,1550,1605]:d.dot(x,975,BLACK)

# Existing loads: both supply wires shown separately.
loads=[(180,340,'ESP32','Pin 5V / VIN · non GPIO'),(650,290,'OLED','Ingresso alimentazione 5 V'),(1050,290,'HX711','VCC a 5 V'),(1450,510,'CIRCUITO AUDIO ESISTENTE','Ingresso 5 V → power-gate → DFPlayer')]
for x,w,title,note in loads:
    cx=x+w/2
    d.box(x,1175,w,155)
    d.wire([(cx,1120),(cx,1175)])
    d.dot(cx,1120)
    d.port(cx,1175)
    d.text(cx,1212,title,23,bold=True,anchor='middle')
    d.text(cx,1246,note,18,anchor='middle')
    d.text(cx,1280,'+5 V in alto · GND in basso',18,anchor='middle')
    d.wire([(cx,1330),(cx,1380)],BLACK)
    d.port(cx,1330,BLACK)
    d.dot(cx,1380,BLACK)
d.wire([(75,1380),(1950,1380)],BLACK,6)
d.dot(75,1380,BLACK)
d.text(180,1097,'+5 V DAL GRUPPO · presenti con S1 chiuso',24,RED,bold=True)
d.footer('Rosso = positivo DC · nero = GND · marrone/blu = L/N, solo lato rete · pallino pieno = fili uniti.',
         'Incrocio senza pallino = nessun collegamento · fusibili per carico massimo 1 A · progetto da montare e collaudare.')
d.save('schema-alimentazione-v50.svg')

# Detail sheet: all low-current connections and passive parts.
q=Drawing('2 / 2 · Resistenze, monitoraggio e collegamenti all’ESP32',
          'È lo stesso circuito della tavola 1. I blocchi ripetuti indicano gli stessi moduli, non componenti aggiuntivi.',1530)

q.box(50,145,940,495,'#edf4ff','#6893bc')
q.text(80,185,'A · Configurazione sui pad del POLOLU 3728',26,bold=True)
q.text(80,222,'R1 e R2 sono esterne; R0 da 15 kΩ è già sulla scheda.',21)
q.text(90,307,'VIN1',24,RED,bold=True)
q.wire([(150,325),(290,325)],RED)
q.wire([(190,325),(190,255),(890,255)],RED)
q.dot(190,325)
q.text(900,263,'MODE',22,RED,bold=True)
q.resistor(290,325,125,'R1','47 kΩ')
q.wire([(415,325),(710,325)],PURPLE)
q.resistor(710,325,120,'R2','1 MΩ')
q.wire([(830,325),(905,325)],PURPLE)
q.text(915,333,'ST',23,PURPLE,bold=True)
q.dot(540,325,PURPLE)
q.text(505,300,'PR1',23,PURPLE,bold=True)
q.box(410,365,265,205,'#e0ebfa','#8ba5c2',dash=True)
q.text(428,395,'INTERNA',18,bold=True)
q.wire([(540,325),(540,425)],PURPLE)
q.box(516,425,48,65,'#fff','#8ba5c2')
q.text(575,448,'R0',22,bold=True)
q.text(575,476,'15 kΩ',22)
q.wire([(540,490),(540,605)],BLACK)
q.text(430,530,'Già',18)
q.text(430,555,'montata.',18)
q.wire([(150,605),(905,605)],BLACK)
q.dot(540,605,BLACK)
q.text(80,612,'GND',21,bold=True)
q.text(700,398,'ST resta collegato',21)
q.text(700,427,'soltanto a R2.',21)

q.box(1030,145,1020,495)
q.text(1060,185,'B · INA219: alimentazione logica e I²C',26,bold=True)
q.box(1080,240,250,280,'#fff')
q.text(1110,275,'INA219',26,bold=True)
q.box(1600,240,400,280,'#fff')
q.text(1630,275,'ESP32 · stessa scheda',25,bold=True)
for y,left,right,c in [(320,'VCC','3V3',ORANGE),(375,'SDA','GPIO32',PURPLE),(430,'SCL','GPIO33',BLUE),(485,'GND','GND',BLACK)]:
    q.text(1110,y+7,left,23,c,bold=True)
    q.wire([(1330,y),(1600,y)],c,4)
    q.port(1330,y,c);q.port(1600,y,c)
    q.text(1630,y+7,right,23,c,bold=True)
q.text(1060,567,'La corrente dei carichi passa soltanto da VIN+ a VIN−: tavola 1.',21)
q.text(1060,606,'VCC dell’INA219 riceve 3,3 V. I²C e GND non passano nello shunt.',21)

q.text(60,708,'C · Lettura della batteria: il relè si chiude quando l’ESP32 ha la sua 3,3 V',27,bold=True)
q.box(60,765,305,140,'#edf8f1','#65a182')
q.text(85,800,'VOLTAIC V50',25,bold=True)
q.text(85,840,'USB-C SUPERIORE',23,bold=True)
q.text(85,875,'Solo misura della batteria',20)
q.wire([(365,820),(440,820)],'#798994',12)
q.text(85,946,'Cavo C ↔ C completo',20)
q.text(85,978,'USB 3.1/3.2',20)
q.box(440,765,285,480,'#f2effa','#a58cc6')
q.text(465,802,'BREAKOUT USB-C',23,bold=True)
q.text(465,836,'presa femmina',21)
q.text(465,893,'SBU1 / A8',23,PURPLE,bold=True)
q.text(465,925,'circa 1,6–2,1 V',21,PURPLE)
q.text(465,1005,'SBU2 / B8: libero',21)
q.text(465,1045,'VBUS: non collegare',20,RED,bold=True)
q.text(465,1080,'alla bilancia.',20,RED,bold=True)
q.text(465,1125,'Altri pad: liberi.',21)
q.text(465,1212,'GND',23,bold=True)
q.port(725,875,PURPLE);q.port(725,1205,BLACK)

# Reed relay: numbered according to top view, internal diode marked.
q.box(840,770,380,395,'#fff8e9','#cfaf72')
q.text(860,805,'K1 · RELÈ REED',25,bold=True)
q.text(860,835,'SIL03-1A72-71D',23,bold=True)
q.wire([(725,875),(930,875)],PURPLE)
q.port(930,875,PURPLE);q.port(1130,875,PURPLE)
q.wire([(930,875),(965,875),(1090,853)],PURPLE)
q.wire([(1110,875),(1280,875)],PURPLE)
q.text(916,912,'1',22,PURPLE,bold=True);q.text(1120,912,'7',22,PURPLE,bold=True)
q.text(876,945,'Contatto NO: aperto da spento',20)
q.text(885,985,'3 (+)',22,ORANGE,bold=True)
q.text(1115,985,'5 (−)',22,BLACK,bold=True)
q.box(963,1000,132,50,'#fff','#c2ab7c')
q.text(1029,1032,'BOBINA',20,bold=True,anchor='middle')
q.wire([(930,1025),(963,1025)],ORANGE)
q.wire([(1095,1025),(1130,1025)],BLACK)
q.port(930,1025,ORANGE);q.port(1130,1025,BLACK)
q.text(860,1120,'Diodo già interno: K → 3, A → 5.',19)
q.text(860,1148,'Numeri pin: vista dall’alto.',19)

q.resistor(1280,875,130,'R3','10 kΩ')
q.wire([(1410,875),(1690,875)],PURPLE)
q.dot(1510,875,PURPLE)
q.wire([(1510,875),(1510,1190)],PURPLE)
q.cap(1510,1190,'C4','100 nF / ≥16 V',False,label_x=1280)
q.wire([(1510,1208),(1510,1330)],BLACK)

q.box(1690,770,340,485)
q.text(1720,812,'ESP32',29,bold=True)
q.text(1720,882,'GPIO36 · ADC',24,PURPLE,bold=True)
q.text(1720,1077,'3V3 · uscita',24,ORANGE,bold=True)
q.text(1720,1197,'GND',24,bold=True)
q.text(1720,935,'5 V e GND di potenza:',19)
q.text(1720,965,'vedi tavola 1.',19)
q.text(1720,1120,'Non è un GPIO.',20,ORANGE,bold=True)
q.port(1690,875,PURPLE)
q.port(1690,1070,ORANGE)
q.port(1690,1190,BLACK)
q.wire([(1690,1070),(790,1070),(790,1025),(930,1025)],ORANGE)
q.wire([(1130,1025),(1180,1025),(1180,1330)],BLACK)
q.wire([(725,1205),(760,1205),(760,1330)],BLACK)
q.wire([(1690,1190),(1630,1190),(1630,1330)],BLACK)
q.wire([(590,1330),(1980,1330)],BLACK,6)
for x in [760,1180,1510,1630]:q.dot(x,1330,BLACK)
q.text(600,1370,'GND comune · stessa linea nera della tavola 1',24,bold=True)
q.footer('C1/C2/C3 sono polarizzati: + sul positivo, − a GND. C4 e R1/R2/R3 non hanno polarità.',
         'K1 serve soltanto alla misura, non interrompe i 5 V della bilancia. Firmware e taratura da adattare prima dell’uso.')
q.save('schema-monitoraggio-v50.svg')
