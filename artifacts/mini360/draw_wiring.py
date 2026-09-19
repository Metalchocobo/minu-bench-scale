"""Generate editable, deterministic wiring illustrations for the actual Mini360."""
from pathlib import Path
from html import escape

ROOT = Path(__file__).resolve().parent
RED, GND, BLUE, GREEN, AMBER = '#c63d36', '#26333d', '#236bb5', '#087d69', '#a66713'
BG = '#f4f7fa'


class Drawing:
    def __init__(self, title, subtitle, h=1300):
        self.h = h
        self.parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="1800" height="{h}" viewBox="0 0 1800 {h}"><title>{escape(title)}</title><rect width="1800" height="{h}" fill="{BG}"/>']
        self.nodes = []
        self.text(55, 53, 'MINÙ / CABLAGGIO ALIMENTAZIONE', 18, GREEN, True)
        self.text(55, 107, title, 36, GND, True)
        self.text(55, 146, subtitle, 21)

    def text(self, x, y, s, size=22, color=GND, bold=False, anchor='start'):
        self.parts.append(f'<text x="{x}" y="{y}" fill="{color}" font-family="Arial,Helvetica,sans-serif" font-size="{size}" font-weight="{700 if bold else 400}" text-anchor="{anchor}">{escape(s)}</text>')

    def rect(self, x, y, w, h, fill='white', stroke='#cbd6df', rx=14):
        self.parts.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}" stroke="{stroke}" stroke-width="2"/>')

    def wire(self, points, color=RED, width=5):
        p = ' '.join(f'{x},{y}' for x, y in points)
        self.parts.append(f'<polyline points="{p}" fill="none" stroke="{BG}" stroke-width="{width+7}" stroke-linejoin="round"/>')
        self.parts.append(f'<polyline points="{p}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round"/>')

    def dot(self, x, y, color=RED):
        self.nodes.append(f'<circle cx="{x}" cy="{y}" r="6" fill="{color}"/>')

    def port(self, x, y, color=RED):
        self.nodes.append(f'<circle cx="{x}" cy="{y}" r="6" fill="white" stroke="{color}" stroke-width="3"/>')

    def cap(self, x, top, bottom, label, polar=True):
        mid = (top + bottom) / 2
        self.wire([(x, top), (x, mid-9)])
        self.wire([(x-18, mid-9), (x+18, mid-9)], GND, 4)
        self.wire([(x-18, mid+9), (x+18, mid+9)], GND, 4)
        self.wire([(x, mid+9), (x, bottom)], GND)
        if polar:
            self.text(x+25, mid-12, '+', 22, RED, True)
        self.text(x+40, mid+6, label, 19)
        self.dot(x, top)
        self.dot(x, bottom, GND)

    def footer(self, note):
        self.text(55, self.h-65, 'Rosso = positivo DC   •   Nero = negativo / GND   •   Pallino pieno = collegamento', 20)
        self.text(55, self.h-29, note, 18)

    def save(self, name):
        (ROOT/name).write_text('\n'.join(self.parts+self.nodes+['</svg>']), encoding='utf-8')


def mini_rear(d, x, y, w=400, h=260):
    """Seller rear view: text upright, arrow left, negative pads at the top."""
    d.rect(x, y, w, h, '#136641', '#0e4f33', 14)
    for px, py in [(x+30,y+35),(x+30,y+h-35),(x+w-30,y+35),(x+w-30,y+h-35)]:
        d.rect(px-15, py-19, 30, 38, '#d5c783', '#e9dea5', 5)
    d.text(x+w/2, y+78, '←', 62, '#f0f5df', True, 'middle')
    d.text(x+w/2, y+148, 'MINI-360', 32, '#f0f5df', True, 'middle')
    d.text(x+w/2, y+188, 'HW-187', 19, '#d5e5ca', False, 'middle')
    d.text(x+60, y+42, 'OUT−', 22, 'white', True)
    d.text(x+60, y+h-28, 'OUT+', 22, 'white', True)
    d.text(x+w-65, y+42, 'IN−', 22, 'white', True, 'end')
    d.text(x+w-65, y+h-28, 'IN+', 22, 'white', True, 'end')


def power_sheet():
    d = Drawing('1 / 3 · Dal pacco batteria ai 5 V della bilancia',
                'Mini360 DAOKAI con MP1482DS · schema dei collegamenti; pad nella tavola 2.', 1420)
    d.rect(55, 185, 1690, 82, '#e6f4ef', '#83b6a0')
    d.text(80, 219, 'RICARICA: USB ESTERNA → USB-C DEL PACCO → CARICATORE INTERNO ALLA BATTERIA', 22, GREEN, True)
    d.text(80, 248, 'NASTIMA BK06-LF60-NATC · ingresso USB dichiarato 5 V / 1,5 A · ricarica indipendente da S1.', 20)

    # Battery is a symbolic block: do not imply a physical terminal location.
    d.rect(65, 350, 255, 300, '#283845', '#1c2932')
    d.rect(85, 370, 215, 130, '#e7edf1', '#e7edf1')
    d.text(192, 410, 'NASTIMA LiFePO4', 21, GND, True, 'middle')
    d.text(192, 447, '6,4 V / 6 Ah', 23, GND, True, 'middle')
    d.text(192, 480, 'BK06-LF60-NATC', 17, GND, False, 'middle')
    d.text(110, 550, 'Morsetto +', 23, '#ffffff', True)
    d.text(110, 615, 'Morsetto −', 23, '#ffffff', True)
    d.port(320, 540)
    d.port(320, 605, GND)

    d.text(575,305,'S1 dopo VIN− dell’INA219, prima del Mini360 e dei condensatori di ingresso',22,GREEN,True)
    # Source fuse, high-side current monitor, then single-pole switch.
    d.wire([(320,540),(370,540)])
    d.rect(370, 519, 115, 42, '#fff4df', '#d1a362', 5)
    d.text(427, 548, 'F1', 22, AMBER, True, 'middle')
    d.text(355, 484, 'F1 vicino al +', 19)
    d.text(360, 586, 'T2 A · proposta¹', 19)
    d.wire([(485,540),(575,540)])
    d.rect(575, 390, 290, 260, '#e6f4ef', '#83b6a0')
    d.text(720, 430, 'INA219', 29, GREEN, True, 'middle')
    d.text(600, 479, 'VIN+', 23, RED, True)
    d.text(778, 479, 'VIN−', 23, RED, True)
    d.wire([(575,540),(675,540)])
    d.rect(675, 525, 85, 30, '#f7f0de', '#aa9c72', 3)
    d.text(717, 517, 'shunt', 17, GREEN, False, 'middle')
    d.wire([(760,540),(865,540),(885,540)])
    d.port(885,540); d.port(935,540)
    d.wire([(885,540),(928,500)])
    d.wire([(935,540),(1000,540)])
    d.text(896,590,'S1',21,GREEN,True)
    d.text(600, 604, 'Positivo entra → esce', 20)
    d.port(575,540); d.port(865,540)
    d.port(720,650,GND)
    d.text(700, 632, 'GND', 17)

    # Generic port block; exact seller view is isolated on the second sheet.
    d.rect(1000, 390, 325, 260, '#e6f4ef', '#83b6a0')
    d.text(1162, 429, 'MINI360', 30, GREEN, True, 'middle')
    d.text(1162, 460, 'MP1482DS', 21, GREEN, True, 'middle')
    d.text(1162, 491, 'Montaggio: 5,11 V', 19, GND, True, 'middle')
    d.text(1018, 520, 'IN+', 21, RED, True)
    d.text(1308, 520, 'OUT+', 21, RED, True, 'end')
    d.text(1018, 594, 'IN−', 21, GND, True)
    d.text(1308, 594, 'OUT−', 21, GND, True, 'end')
    d.port(1000,540); d.port(1325,540)
    d.port(1000,605,GND); d.port(1325,605,GND)
    d.text(1000, 357, 'Schema funzionale, non vista del PCB', 18)

    # Output remains connected: S1 does not isolate the buck from USB-PC power.
    d.wire([(1325,540),(1730,540),(1730,950),(180,950)])
    d.text(1400, 436, 'OUT+ diretto ai carichi', 24, GREEN, True)
    d.text(1400, 478, 'Nessun interruttore in uscita', 19)
    d.text(1400, 581, 'USB PC: ritorno possibile', 20, AMBER, True)
    d.text(1400, 617, 'S1 OFF stacca la batteria,', 17)
    d.text(1400, 644, 'ma OUT+ resta collegato all’ESP32.', 17)
    d.text(1400, 677, 'Comportamento USB non verificato.', 17)
    d.text(1400, 710, 'ON + PC: sorgenti non isolate da S1.', 17)

    # DC return, with all branches explicitly connected.
    d.wire([(320,605),(340,605),(340,830),(1570,830)], GND, 6)
    d.wire([(720,650),(720,830)], GND)
    d.wire([(1000,605),(970,605),(970,830)], GND)
    d.wire([(1325,605),(1360,605),(1360,830)], GND)
    for x in (340,720,970,1360): d.dot(x,830,GND)
    d.text(370, 868, 'NEGATIVO COMUNE / GND · distinto da VIN− dell’INA219', 23, GND, True)
    d.text(390, 690, 'VIN− è ancora il positivo!', 22, RED, True)

    # Existing project decoupling parts; branches do not carry signal currents.
    d.dot(960,540)
    d.wire([(960,540),(960,738),(545,738)])
    d.cap(550,738,830,'220 µF / 16 V')
    d.cap(790,738,830,'100 nF',False)
    d.text(370, 724, 'INGRESSO MINI360', 20, GREEN, True)
    d.text(750, 724, 'C1 / C2', 20, GREEN, True)
    d.text(220, 920, 'LINEA 5 V NOMINALI · regolazione riferita 5,11 V · carichi su OUT+', 23, RED, True)

    # Loads and output capacitors.
    d.wire([(340,830),(90,830),(90,1195),(1690,1195)],GND,6)
    load_specs = [(175,270,'ESP32','Pin VIN / 5V'),(505,245,'HX711','VCC'),(810,245,'OLED','VCC compatibile 5 V'),(1115,310,'AUDIO ESISTENTE','Power-gate → DFPlayer')]
    for x,w,title,note in load_specs:
        cx=x+w/2
        d.rect(x,1010,w,125)
        d.wire([(cx,950),(cx,1010)])
        d.dot(cx,950); d.port(cx,1010)
        d.text(cx,1051,title,24,GND,True,'middle')
        d.text(cx,1087,note,19,GND,False,'middle')
        d.text(cx,1116,'GND in basso',17,GND,False,'middle')
        d.wire([(cx,1135),(cx,1195)],GND)
        d.dot(cx,1195,GND); d.port(cx,1135,GND)
    d.cap(1485,950,1195,'C3',True)
    d.text(1445,907,'USCITA MINI360',22,GREEN,True)
    d.text(1445,935,'C3 / C4 tra OUT+ e OUT−',18)
    d.text(1525,1135,'470 µF',18)
    d.text(1525,1165,'10 V',18)
    d.cap(1640,950,1195,'C4',False)
    d.text(1660,1142,'100 nF',17)
    d.text(175, 1245, '¹ F1 T2 A: proposta per fili di potenza corti ≥ 0,5 mm²; verificare gli spunti del carico reale.', 20)
    d.text(175, 1279, 'Condensatori: saldare vicino ai pad del Mini360. Vedi il dettaglio 03-condensatori-mini360.', 20)
    d.footer('Incroci senza pallino = fili separati · 18/09/2026 · Collegamenti logici, non distanze di montaggio.')
    d.save('01-alimentazione-mini360.svg')


def detail_sheet():
    d = Drawing('2 / 3 · Pad del Mini360, regolazione e INA219',
                'DAOKAI B0B82GL5XN · MP1482DS identificato in foto; vista del retro da istruzioni del venditore.', 1450)
    d.rect(55,185,1690,550)
    d.text(85,225,'A · GUARDA IL RETRO: SCRITTA MINI-360 DRITTA E FRECCIA VERSO SINISTRA',23,GREEN,True)
    mini_rear(d,695,310,410,270)

    d.text(1180,324,'INGRESSO DALLA BATTERIA',22,GND,True)
    d.text(1180,364,'IN− → negativo / GND',22)
    d.text(1180,560,'IN+ → positivo dopo S1',22,RED,True)
    d.wire([(1075,345),(1150,345)],GND)
    d.wire([(1075,545),(1150,545)])
    d.text(80,324,'USCITA VERSO LA BILANCIA',22,GND,True)
    d.text(80,364,'OUT− → GND comune',22)
    d.text(80,560,'OUT+ → linea +5 V',22,RED,True)
    d.wire([(470,345),(725,345)],GND)
    d.wire([(470,545),(725,545)])
    for x,y,c in [(1075,345,GND),(1075,545,RED),(725,345,GND),(725,545,RED)]:d.port(x,y,c)
    d.text(900,623,'Saldare sui quattro pad dorati. Non scambiare IN con OUT.',21,GND,True,'middle')
    d.text(900,661,'Sul fronte la posizione appare specchiata: questa tavola mostra soltanto il RETRO.',20,GND,False,'middle')
    d.text(900,698,'Schema ingrandito · scheda circa 17 × 12 mm · seguire anche la serigrafia dell’esemplare.',19,GND,False,'middle')

    d.rect(55,765,790,540)
    d.text(85,810,'B · PRIMA REGOLA, POI COLLEGA IL CARICO',23,GREEN,True)
    for y,num,line in [(855,'1','PC scollegato; filo OUT+ verso i carichi staccato.'),(899,'2','Collega IN+/IN−; porta S1 su ON.'),(943,'3','Tester in volt DC: rosso OUT+, nero OUT−.'),(987,'4','Riferimento 5,00 V; montaggio regolato a 5,11 V.'),(1031,'5','OFF e batteria scollegata: completa il filo OUT+.'),(1075,'6','Ricollega la batteria; ON e verifica sotto carico.')]:
        d.text(85,y,num,25,GREEN,True)
        d.text(125,y,line,20)
    d.rect(90,1110,255,120,'#283845','#283845')
    d.rect(113,1131,210,63,'#d6e7c0','#d6e7c0',4)
    d.text(218,1178,'5.11 V',36,GND,True,'middle')
    d.text(385,1146,'Valore riferito sul montaggio.',22)
    d.text(385,1182,'Il trimmer è sul lato componenti.',19)
    d.text(85,1265,'Buck soltanto: vicino a 5 V in ingresso può perdere la regolazione.',19,AMBER,True)

    d.rect(885,765,860,540)
    d.text(915,810,'C · INA219 → ESP32',25,GREEN,True)
    d.text(915,851,'È lo stesso INA219 della tavola 1.',20)
    d.rect(925,881,210,290,'#e6f4ef','#83b6a0')
    d.rect(1430,881,270,290,'#edf3fa','#a0b8d3')
    d.text(1030,917,'INA219',24,GREEN,True,'middle')
    d.text(1565,917,'ESP32',24,BLUE,True,'middle')
    for y,left,right,c in [(957,'VCC','3V3',AMBER),(1014,'SDA','GPIO32',GREEN),(1071,'SCL','GPIO33',BLUE),(1128,'GND','GND',GND)]:
        d.text(950,y+8,left,22,c,True)
        d.wire([(1135,y),(1430,y)],c,4)
        d.port(1135,y,c);d.port(1430,y,c)
        d.text(1460,y+8,right,22,c,True)
    d.text(915,1215,'La corrente di potenza passa da VIN+ a VIN−.',21)
    d.text(915,1254,'SDA/SCL e VCC non portano la corrente della bilancia.',20)
    d.footer('Fonte pad: DAOKAI 71XIQphK08L · primo funzionamento positivo riferito da Andrea; nessuna misura dei transitori.')
    d.save('02-pad-regolazione-ina219.svg')


def capacitor_sheet():
    d = Drawing('3 / 3 · Condensatori in ingresso e in uscita',
                'Collegamenti in parallelo ai morsetti del Mini360. Nel montaggio reale, connessioni corte vicino ai pad.', 1020)
    d.rect(55,215,650,510,'#edf3fa','#a0b8d3')
    d.rect(1095,215,650,510,'#e6f4ef','#83b6a0')
    d.text(85,263,'INGRESSO · tra IN+ e IN−',29,BLUE,True)
    d.text(85,304,'C1 + C2 · lato batteria, dopo S1',22)
    d.text(1125,263,'USCITA · tra OUT+ e OUT−',29,GREEN,True)
    d.text(1125,304,'C3 + C4 · lato 5 V della bilancia',22)
    d.rect(760,350,280,330,'#283845','#1c2932')
    d.text(900,469,'MINI360',32,'white',True,'middle')
    d.text(900,512,'5,11 V',28,'white',True,'middle')
    d.text(900,570,'Schema funzionale',18,'white',False,'middle')
    d.text(900,600,'Pad reali: tavola 2',18,'white',False,'middle')
    d.wire([(110,390),(760,390)])
    d.wire([(110,640),(760,640)],GND)
    d.wire([(1040,390),(1690,390)])
    d.wire([(1040,640),(1690,640)],GND)
    for x,y,c in [(760,390,RED),(760,640,GND),(1040,390,RED),(1040,640,GND)]:
        d.port(x,y,c)
    d.text(780,413,'IN+',23,'white',True)
    d.text(780,632,'IN−',23,'white',True)
    d.text(1020,413,'OUT+',23,'white',True,'end')
    d.text(1020,632,'OUT−',23,'white',True,'end')
    for x,ref,value,rating,kind,polar in [
        (135,'C1','220 µF','16 V','Elettrolitico',True),
        (455,'C2','100 nF','0,1 µF · 104','Ceramico',False),
        (1150,'C3','470 µF','10 V','Elettrolitico',True),
        (1470,'C4','100 nF','0,1 µF · 104','Ceramico',False),
    ]:
        d.cap(x,390,640,'',polar)
        d.text(x+46,468,ref,23,GND,True)
        d.text(x+46,517,value,28,GND,True)
        d.text(x+46,555,rating,21)
        d.text(x+46,594,kind,19)
    d.text(85,692,'C1: + su IN+ / − su IN−',21,BLUE,True)
    d.text(1125,692,'C3: + su OUT+ / − su OUT−',21,GREEN,True)
    d.wire([(730,640),(730,750),(1070,750),(1070,640)],GND)
    d.dot(730,640,GND); d.dot(1070,640,GND)
    d.text(900,788,'IN− e OUT−: GND comune',20,GND,True,'middle')
    d.text(80,850,'MONTAGGIO: direttamente sui pad se ci stanno, oppure subito accanto con collegamenti corti.',25,GREEN,True)
    d.text(80,890,'I ceramici non hanno polarità. La distanza nel disegno serve solo a leggere i collegamenti.',22)
    d.footer('Dettaglio dello stesso cablaggio della tavola 1 · 18/09/2026 · Nessuna modifica del circuito di ricarica.')
    d.save('03-condensatori-mini360.svg')


if __name__ == '__main__':
    power_sheet()
    detail_sheet()
    capacitor_sheet()
