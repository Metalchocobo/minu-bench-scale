"""Build a seven-page A3 landscape guide from the checked assembly illustrations."""
from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A3, landscape
from reportlab.lib.utils import ImageReader

ROOT = Path(__file__).resolve().parent

if __name__ == '__main__':
    target = ROOT/'guida-illustrata.pdf'
    width, height = landscape(A3)
    document = canvas.Canvas(str(target), pagesize=(width, height), pageCompression=1)
    document.setTitle('Minù · Cablaggio corrente NASTIMA + Mini360')
    document.setAuthor('Minù')
    document.setSubject('Guida di montaggio: sette tavole con pin, fili, resistenze e rilevamento USB PC817.')
    titles = ['Tutta la bilancia', 'Alimentazione', 'HX711 e cella', 'Audio', 'Tastiera, LED e buzzer', 'OLED e INA219', 'USB e PC817']
    plates = sorted(ROOT.glob('0[1-7]-*.png'))
    if len(plates) != 7:
        raise ValueError('Expected exactly seven checked PNG plates.')
    for number, name in enumerate(plates):
        document.bookmarkPage(name.stem)
        document.addOutlineEntry(titles[number], name.stem)
        document.drawImage(ImageReader(str(name)), 12, 12, width-24, height-24, preserveAspectRatio=True, anchor='c')
        document.showPage()
    document.save()
    print(target.name)
