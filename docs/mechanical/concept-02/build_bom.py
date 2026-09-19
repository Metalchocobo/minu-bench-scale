"""Join the actual CAD quantity take-off to researched procurement references."""
from pathlib import Path
import csv
import json

OUT = Path(__file__).resolve().parent
geometry = json.loads((OUT/'bom-geometry.json').read_text(encoding='utf-8'))
load = json.loads((OUT/'component-specs-load.json').read_text(encoding='utf-8'))
controls = json.loads((OUT/'component-specs-controls.json').read_text(encoding='utf-8'))
cell = load['load_cell']
references = {
 'ZEMIC_L6D_C3_30KG_0_4B': (cell['purchase_url'], 'Essmann 11250496', 'Cella 30 kg; carico utile di progetto 20 kg, accuratezza 1 g non provata.'),
 'ELESA_531051': (load['feet']['purchase_url'], 'Elesa 531051', '4 piedi completi di pattino; filetto M6 nel basamento.'),
 'SCREW_M6x25_ISO4762': (load['fasteners'][0]['source_url'], 'Keller 240912625, 8.8', 'Lato fisso della cella; rondella ISO7089.'),
 'SCREW_M6x45_DIN7991': (load['fasteners'][1]['source_url'], 'Accu SSK-M6-45-10.9-Z', 'Lato mobile; testa D12, cono90. Coppia e giunto da confermare con Zemic.'),
 'WASHER_M6_ISO7089': (load['fasteners'][2]['source_url'], 'ISO7089 M6 200HV', '6,4 x12 x1,6 mm.'),
 'NUT_M6_DIN934': (load['fasteners'][3]['source_url'], 'M6 DIN934 zincato', 'Chiave10, altezza5. Acquistabile a standard anche in ferramenta.'),
 'NUT_M3_DIN934': ('https://www.westfieldfasteners.co.uk/Metric-Nuts/Hex-Nut-M3-Bright-Zinc-Plate-Class-8.html', 'Westfield WF15111', 'Chiave5,5, altezza2,4; sedi captive stampate.'),
 'SCREW_M3x10_DIN7991': ('https://www.orbitalfasteners.co.uk/products/m3x10-csk-skt-screw-stst-a4-din-7991', 'Orbital 5500020 A4', 'Testa D6; svasatura90; lunghezza totale10.'),
 'SCREW_M3x8_ISO4762': ('https://www.westfieldfasteners.co.uk/A4-ScrewBolt-SHCap-M3.html', 'Westfield WF14272 A4', 'Selezionare M3 x8, testa D5,5 x3.'),
 'SCREW_M3x10_ISO4762': ('https://www.westfieldfasteners.co.uk/A4-ScrewBolt-SHCap-M3.html', 'Westfield WF14261 A4', 'Selezionare M3 x10, testa D5,5 x3.'),
 'SCREW_M3x16_ISO4762': ('https://www.westfieldfasteners.co.uk/Bolts-Screws-Metric/A4-Socket-Head-Cap-Screw-M3x16mm.html', 'Westfield WF14263 A4', 'Testa D5,5 x3.'),
 'SETSCREW_M6x60_DIN913_FLAT_END': ('https://www.rwproducts.nl/din-913-stelschroef-binnenzeskant-met-afschuining-elvz-m6x60', 'RW D913V060060, DIN913 45H', '4 fermi regolabili; gioco di progetto da tarare sul prototipo.'),
 'KEYPAD_1x4_BERRYBASE_B_SM4T': ('https://www.berrybase.de/en/membrane-keypad-4-keys-without-labelling-with-adhesive-layer', 'BerryBase B-SM4T', '2 membrane adesive70x20x1, totale8tasti. Pinout/coda da campione. Origine Cina.'),
 'USB_C_PANEL_6069': ('https://www.adafruit.com/product/6069', 'Premier PCM-0726 / Adafruit6069', 'Prolunga USB-C300mm, include dado, O-ring e tappo. Alternativa diretta Cina in research-controls.md.'),
 'REUSE_OLED_SSD1322': ('', 'Posseduto: SSD1322 SPI256x64 3,12pollici', 'Non acquistare. Ingombro e serraggio carrier da adattare al modulo reale.'),
}
for purchase in controls['additional_purchases']:
    references[purchase['bom']] = (
        purchase['url'],
        purchase.get('preferred_sku', purchase.get('sku', purchase.get('wire_sku', purchase['bom']))),
        purchase.get('assembly_note', purchase.get('note', purchase.get('cad_note', 'Quote dei connettori da verificare sul campione.'))),
    )
references['USB_A_OUTPUT_HARNESS_2697'] = (
    'https://www.adafruit.com/product/2697', 'Adafruit 2697, 22 AWG',
    'Un cavo da 1 m: conservare USB-A, accorciare a circa 200 mm ed eliminare il jack. Terminare sulla distribuzione 5 V posseduta dopo verifica elettrica.'
)
design_notes = {
 'CNC_BASE_300x320x6': 'Faccia superiore piana. Fori filettati rappresentati al diametro nominale; usare le specifiche di maschiatura nelle note CNC.',
 'CNC_CELL_FIXED_SPACER': 'Distanziale fisso da 6 mm; superfici di serraggio piane e sbavate secondo le note CNC.',
 'CNC_CELL_MOVING_SPACER': 'Distanziale mobile da 22 mm; mantiene il portapiatto separato dal guscio.',
 'CNC_CARRIER_276x196x8': 'Due svasature M6 e sei filetti M3. Sostiene quasi tutta la superficie inferiore del piatto plastico.',
 'PRINT_OUTER_SHELL': 'Guscio rimovibile, pareti da 3 mm e sedi per dadi M3. Giochi delle sedi da verificare con lo stampatore.',
 'PRINT_PAN_280x200x4': 'Piatto da stampare, sostenuto dal portapiatto metallico; sei viti M3 svasate.',
 'PRINT_POWER_TRAY': 'Vassoio isolante rimovibile del gruppo UPS, con quattro fissaggi al fondo.',
 'PRINT_POWER_RETAINING_STRAP': 'Due staffe trattengono il bordo della lastra acrilica inferiore del kit UPS; stampare entrambi i file, le sagome differiscono.',
 'PRINT_ELECTRONICS_TRAY': 'Superficie piana isolante per le schede possedute. I loro singoli attacchi richiedono le misure dei PCB.',
 'PRINT_DISPLAY_BEZEL': 'Cornice sostituibile; sede del modulo e superficie sicura di serraggio da adattare al display posseduto.',
 'PRINT_DISPLAY_REAR_BAR': 'Due traversini rimovibili. Verificare che il modulo reale abbia bordi liberi sui quali serrare.',
 'PRINT_CABLE_CLAMP_LOWER': 'Due guide apribili per i percorsi USB; usare i file distinti per le due posizioni.',
 'PRINT_CABLE_CLAMP_UPPER': 'Coperchi delle guide con quattro viti M3 complessive. Diametro e curvatura dei cavi reali da verificare.',
 'PRINT_KEYPAD_CABLE_HOOD': 'Due ponti coprono le uscite delle code; appoggi adesivi esterni alle membrane, senza pressione sulle zone dei tasti.',
 'CUT_3M468MP_KEYPAD_HOOD_PAD': 'Ritagliare due piazzole 24×2 e quattro 2×6 mm. Spessore 0,13 mm; adesione sulla finitura stampata da verificare.',
 'CRIMP_CONTACT_2p54': 'Sei contatti femmina per fili AWG22–28; verificare ritenuta nel contenitore DUPH-1X06 e crimpatura sul filo AWG26 selezionato.',
}

rows=[]
ups_names=[]
for entry in geometry:
    item=entry['item']
    if 'UPS_VENDOR' in item or item.startswith('WAVESHARE_UPS_HAT'):
        ups_names.extend(entry['cad_names'])
        continue
    action='Acquistare'
    order_quantity=entry['quantity']
    order_unit='pezzi'
    source,sku,note=references.get(item,(entry.get('source',''),item,entry.get('notes','')))
    note=design_notes.get(item,note)
    if item.startswith('PRINT_'):
        action='Stampare su file';source='https://jlc3dp.com/';sku='PA12 MJF/SLS, pezzo a disegno'
    elif item.startswith('CNC_'):
        action='Lavorare su file';source='https://jlccnc.com/';sku='6061-T6, pezzo a disegno'
    elif item=='CUT_DISPLAY_WINDOW':
        action='Tagliare su file';sku='PMMA trasparente 1,5 mm';source='https://jlccnc.com/';note='Richiedere taglio/fresatura82x24x1,5; tolleranza e disponibilita lastra da preventivo.'
    elif item.startswith('CUT_3M'):
        action='Consumabile da ritagliare';sku='3M468MP, spessore 0,13 mm'
    elif item.startswith('REUSE_'):
        action='Riutilizzare'
    elif item.startswith('INCLUDED_'):
        action='Incluso nel componente';order_quantity=0;note='Non acquistare separatamente. '+note
    elif entry['category']=='cable':
        action='Cablaggio da assemblare';note='Percorso CAD indicativo; lunghezza e terminali secondo montaggio. '+note
    if ('USB_A' in item or 'OUTPUT_HARNESS' in item) and entry['category']=='cable':
        source='https://www.adafruit.com/product/2697';sku='Adafruit2697 22AWG da accorciare'
        action='Incluso nel cablaggio';order_quantity=0;note='Ricavato dal cavo Adafruit 2697 già elencato; non acquistare un secondo cavo. '+note
    if item=='CUSTOM_5V_TERMINATION':
        action='Riutilizzare / adattare';order_quantity=0;source='';sku='Interfaccia alla distribuzione 5 V posseduta'
        note='Segnaposto di interfaccia, non un morsetto commerciale selezionato. Sagoma e innesto da adattare alla scheda reale.'
    if item.startswith('REUSE_'):
        order_quantity=0
    if item=='HEADER_1x5_2p54':
        order_quantity=1;order_unit='strip da 20 poli'
        note='Tagliare due pettini da 5 poli da una sola strip. Verificare profondità di innesto sul campione; pin inclusi. Alternativa Cina LCSC C2977586 con minimo 5 strip.'
    if item=='CUSTOM_KEYPAD_MATRIX_BRANCH':
        order_quantity=1;order_unit='nastro 10 fili da 1 m'
        note='Ricavare due rami da 5 fili, taglio iniziale 200 mm ciascuno. Unire le quattro righe a monte dei sei contatti; saldare e isolare i giunti, non inserire due fili nello stesso crimp non specificato. Adeguare le lunghezze al montaggio.'
    if item=='CUT_3M468MP_KEYPAD_HOOD_PAD':
        order_quantity=1;order_unit='ritaglio per 6 piazzole'
    rows.append(dict(id=item,quantity=entry['quantity'],order_quantity=order_quantity,order_unit=order_unit,description=entry['label'],action=action,sku=sku,url=source,notes=note,cad_parts='; '.join(entry['cad_names'])))
if ups_names:
    rows.append(dict(id='WAVESHARE_UPS_HAT_D_KIT',quantity=1,description='Kit USB-C UPS HAT(D), due21700, protezione e viti',action='Acquistare',sku='Waveshare25507 CN',url='https://www.waveshare.net/shop/UPS-HAT-D.htm',notes=f'Un kit; {len(ups_names)} solidi del relativo gruppo nel CAD. EN25567 alternativo senza batterie. Variante batteria proposta, non ancora provata con firmware.',cad_parts='; '.join(ups_names)))
rows.append(dict(id='EXTERNAL_USB_C_PSU',quantity=1,description='Alimentatore USB-C esterno15W spina EU',action='Acquistare se necessario',sku='Raspberry Pi USB-C5,1V3A',url='https://www.raspberrypi.com/products/power-supply/',notes='Esterno alla scocca; non fa parte degli STL.',cad_parts=''))
rows.append(dict(id='DISPLAY_ADHESIVE',quantity=1,description='Adesivo sottile per perimetro vetrino',action='Consumabile',sku='3M467MP, ritaglio',url='https://www.3m.com/3M/en_US/p/dc/v000185218/',notes='Spessore tipico0,05-0,06mm; adesione sul materiale stampato da verificare. Ritagliare senza coprire apertura80x22.',cad_parts='display_window'))
rows.append(dict(id='KEY_LABEL_OVERLAY',quantity=2,description='Etichette funzioni per tastierini blank',action='Stampare grafica',sku='Strisce grafiche su misura70x20mm',url='',notes='SKIP/WIFI/SLEEP/CLEAR e ENTER/TOTAL/MODE/TARE; allineare le zone sensibili al campione. Grafica illustrativa nel viewer.',cad_parts='keypad_strip_1; keypad_strip_2'))
for row in rows:
    row.setdefault('order_quantity', row['quantity'])
    row.setdefault('order_unit', 'pezzi')
    if row['order_quantity']==1 and row['order_unit']=='pezzi':
        row['order_unit']='pezzo'
(OUT/'distinta-acquisti.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')
with (OUT/'distinta-acquisti.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
md=['# Distinta componenti e lavorazioni - Proposta 02','',
    'Quantità derivate dall’assieme CAD. Ricerca del 7 settembre 2026; nessun ordine effettuato. Prezzi, spedizione in Italia e disponibilità richiedono conferma al momento dell’acquisto. I link sono riferimenti concreti di prodotto o lavorazione, non un carrello già verificato. Le quantità da ordinare tengono conto dei componenti inclusi e delle strip da tagliare.','',
    'I sottopezzi inclusi nei kit sono elencati per riconoscere i solidi del CAD: **non vanno acquistati due volte**. Il display e le schede gia presenti vengono riutilizzati.','',
    '| Q.tà CAD | Da ordinare / produrre | Componente / codice | Azione | Fonte |','|---:|---|---|---|---|']
for r in rows:
    link=f"[Prodotto / servizio]({r['url']})" if r['url'] else 'Su misura / posseduto'
    order=f"{r['order_quantity']} {r['order_unit']}" if r['order_quantity'] else '—'
    md.append(f"| {r['quantity']} | {order} | {r['description']} — `{r['sku']}` | {r['action']} | {link} |")
md += ['', '## Note di assemblaggio e acquisto', '']
for r in rows:
    if r['notes'] and r['action']!='Incluso nel componente':md.append(f"- **{r['id']}**: {r['notes']}")
md += ['', '## Parti gia presenti', '',
       'ESP32 DevKit, HX711, INA219, DFPlayer Mini, buzzer e altoparlante restano quelli del progetto. Sono previsti spazio e vassoio isolante rimovibile; i singoli attacchi PCB richiedono le misure delle varianti possedute. La batteria SLA, CTK3S e buck appartengono alla configurazione attuale e non sono compresi nella variante USB-C al litio proposta.', '',
       'Il piatto e i supporti sono **pezzi a disegno**: stampare le plastiche e lavorare le quattro parti metalliche dai file allegati. Consultare manufacturing-notes.md per filetti e svasature: i cilindri nominali nel STEP non indicano il diametro di preforo per maschiatura.']
(OUT/'distinta-acquisti.md').write_text('\n'.join(md)+'\n',encoding='utf-8')
print(json.dumps({'procurement_rows':len(rows),'CAD_groups':len(geometry),'UPS_geometry_parts':len(ups_names)}))
