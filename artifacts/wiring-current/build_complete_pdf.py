"""Export the complete assembly map as one landscape A2 page."""
from pathlib import Path
from io import BytesIO
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A2, landscape
from reportlab.lib.utils import ImageReader

ROOT=Path(__file__).resolve().parent

if __name__ == '__main__':
    target=ROOT/'cablaggio-generale-pc817.pdf'
    width,height=landscape(A2)
    buffer=BytesIO()
    document=canvas.Canvas(buffer,pagesize=(width,height),pageCompression=1)
    document.setTitle('Minù · Cablaggio generale con ESP32 a 30 pin')
    document.setAuthor('Minù')
    document.setSubject('Tutti i fili su un foglio A2, con pin ESP32 e rilevamento alimentatore USB tramite PC817.')
    document.drawImage(ImageReader(str(ROOT/'00-wiring-complete.png')),12,12,width-24,height-24,preserveAspectRatio=True,anchor='c')
    document.showPage()
    document.save()
    temporary=ROOT/'.cablaggio-generale-pc817.tmp.pdf'
    temporary.write_bytes(buffer.getvalue())
    temporary.replace(target)
    print(target.name)
