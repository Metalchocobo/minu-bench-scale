"""Generate the USB-presence detector plate; leave the other plates untouched.

PC817 DIP-4 pin numbers are a top view, with the package notch facing up.
Pinout reference: SHARP PC817XxNSZ1B datasheet, page 2:
https://global.sharp/products/device/lineup/data/pdf/datasheet/PC817XxNSZ1B_e.pdf
The two electrical sides remain distinct in the detector. This does not
establish the unknown internal ground topology of the NASTIMA pack.
"""

import sys
sys.dont_write_bytecode = True
from draw_wiring import BLUE, INK, MUTED, ORANGE, PAPER, RED, Plate


def pc817_top_view(d, x=790, y=560):
    """Draw the DIP-4 top view and return its numbered lead coordinates."""
    width, height = 320, 300
    pins = {
        1: (x - 40, y + 90),
        2: (x - 40, y + 240),
        4: (x + width + 40, y + 90),
        3: (x + width + 40, y + 240),
    }
    d.rect(x, y, width, height, '#344650', '#25323a', 15)
    # The notch is part of the package outline, not an electrical terminal.
    d.parts.append(
        f'<path d="M{x+130} {y} A30 30 0 0 0 {x+190} {y}" '
        f'fill="{PAPER}" stroke="{INK}" stroke-width="3"/>'
    )
    d.circle(x + 23, y + 25, 8, PAPER, PAPER)
    for number, color in ((1, RED), (2, INK), (4, BLUE), (3, INK)):
        px, py = pins[number]
        edge = x if number in (1, 2) else x + width
        d.wire([(px, py), (edge, py)], color)
        d.port(px, py, color=color)
    for tx, ty, label, anchor in (
        (x + 13, y + 67, '1  ANODO', 'start'),
        (x + 13, y + 217, '2  CATODO', 'start'),
        (x + width - 13, y + 67, '4  COLLETTORE', 'end'),
        (x + width - 13, y + 217, '3  EMETTITORE', 'end'),
    ):
        d.text(tx, ty, label, 18, PAPER, True, anchor)
    d.text(x + width / 2, y + 146, 'PC817', 34, PAPER, True, 'middle')
    d.text(x + width / 2, y + 179, 'DIP-4', 22, '#d6e3e9', False, 'middle')
    d.text(x + width / 2, y - 25, 'VISTA DALL’ALTO · TACCA IN ALTO', 20, INK, True, 'middle')
    d.text(x + width / 2, y + height + 28, 'Il puntino identifica il pin 1', 20, MUTED, False, 'middle')
    return pins


def plate():
    d = Plate(
        7,
        'Presenza USB con PC817',
        'USB esterna al pacco → optoisolatore → GPIO36 / VP. USB presente = LOW; USB assente = HIGH.',
    )
    d.note(
        65, 174, 1670, 'Punto di prelievo: cavo USB di ricarica, soltanto 5 V DC',
        ['Preleva USB +5 V e USB − prima dell’ingresso del pacco. Il ramo PC817 è in parallelo al caricatore.'],
        94,
    )

    # Main charging path: neither wire contains the detector's 560-ohm resistor.
    d.board(65, 290, 400, 200, 'USB ESTERNA', 'Alimentatore / cavo · 5 V DC', '#fff1ed')
    d.board(1260, 290, 475, 200, 'USB-C NASTIMA', 'Ingresso del caricatore interno', '#fff1ed')
    d.port(465, 390, '+5 V', RED)
    d.port(465, 450, 'USB −', INK)
    d.port(1260, 390, '+5 V USB', RED, 'left')
    d.port(1260, 450, 'USB −', INK, 'left')
    d.wire([(465, 390), (1260, 390)], RED)
    d.wire([(465, 450), (1260, 450)], INK)
    d.text(860, 319, 'PERCORSO DI RICARICA DIRETTO', 24, INK, True, 'middle')
    d.text(860, 345, 'La resistenza 560 Ω non va su questo percorso.', 21, RED, True, 'middle')

    d.rect(65, 505, 665, 390, '#fff4f1', '#e6c6bd', background=True)
    d.text(90, 544, 'LATO USB · ramo PC817', 24, RED, True)
    d.rect(1180, 505, 555, 390, '#eef5fb', '#c4d5e5', background=True)
    d.text(1202, 541, 'LATO ESP32', 24, BLUE, True)
    pins = pc817_top_view(d)

    # Input LED: +5 V -> 560 ohm -> pin 1, USB return -> pin 2.
    # The input branch crosses the main USB return without joining it.
    d.wire([(550, 390), (550, 650), (565, 650)], RED)
    d.resistor(640, 650, '560 Ω · ¼ W', color=RED)
    d.wire([(715, 650), pins[1]], RED)
    d.dot(550, 390, RED)
    d.wire([(500, 450), (500, 800), pins[2]], INK)
    d.dot(500, 450, INK)
    d.text(90, 610, 'USB +5 V', 24, RED, True)
    d.text(90, 645, '↓ ramo PC817', 22, RED)
    d.text(90, 760, 'USB −', 24, INK, True)
    d.text(90, 832, '560 Ω limita solo la corrente del pin 1.', 21, INK)
    d.text(90, 869, 'La ricarica del pacco prosegue nei due fili sopra.', 21, MUTED)

    # Output transistor: emitter to ESP ground; collector to VP with a pull-up.
    d.board(1440, 505, 295, 375, 'ESP32 DevKit', 'Tre fili sul lato ESP32', '#e7eef5')
    for py, label, color in ((620, '3V3', ORANGE), (770, 'GPIO36 / VP', BLUE), (855, 'GND ESP32', INK)):
        d.port(1440, py, label, color, 'left', 22)
    d.wire([(1440, 620), (1320, 620)], ORANGE)
    d.resistor(1320, 695, '10 kΩ', vertical=True, color=ORANGE)
    d.wire([pins[4], (1215, 650), (1215, 770), (1440, 770)], BLUE)
    d.dot(1320, 770, BLUE)
    d.wire([pins[3], (1245, 800), (1245, 855), (1440, 855)], INK)
    d.text(1202, 585, 'Pull-up esterna', 22, INK, True)
    d.text(1202, 615, 'verso 3V3', 21, ORANGE)

    d.note(65, 925, 510, '1 · Collegamenti del PC817', [
        'USB +5 V → 560 Ω → pin 1 (anodo)',
        'USB − → pin 2 (catodo)',
        'Pin 4 (collettore) → GPIO36 / VP',
        'Pin 3 (emettitore) → GND ESP32',
    ], 200)
    d.note(600, 925, 570, '2 · Resistenza e lettura digitale', [
        '10 kΩ fra GPIO36 / VP e 3V3 ESP32.',
        'USB presente: GPIO36 / VP = LOW.',
        'USB assente: GPIO36 / VP = HIGH.',
        'Rileva la presenza dei 5 V USB esterni.',
    ], 200)
    d.note(1195, 925, 540, '3 · Limiti della lettura', [
        'Nessun ponte USB − / GND ESP32 qui.',
        'Isolamento globale del pacco non stabilito.',
        'Non misura corrente né fine carica.',
        'Collaudo fisico non documentato.',
    ], 200)

    # Use a dedicated legend: the USB return and ESP ground are distinct nets.
    for x, color, label in (
        (65, RED, '+5 V USB'), (390, INK, 'USB −'), (650, ORANGE, '3V3 ESP32'),
        (980, BLUE, 'GPIO36 / VP'), (1350, INK, 'GND ESP32'),
    ):
        d.wire([(x, 1177), (x + 48, 1177)], color)
        d.text(x + 62, 1185, label, 21)
    d.text(
        65, 1224,
        'Pinout: SHARP PC817XxNSZ1B, p. 2, vista dall’alto. ● = unione; incrocio senza punto = nessun collegamento.',
        19, MUTED,
    )
    return d


if __name__ == '__main__':
    plate().save('07-usb-pc817')
    print('Generated 07-usb-pc817.svg.')
