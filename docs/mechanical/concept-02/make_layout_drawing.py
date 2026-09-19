"""Nominal layout and load-path drawing; not a replacement for the CNC notes."""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle, Circle

OUT=Path(__file__).resolve().parent/'drawings'
OUT.mkdir(exist_ok=True)
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'svg.fonttype':'none'})
fig=plt.figure(figsize=(15,9),facecolor='white')
fig.suptitle('MINÙ  /  Proposta 02',x=.06,y=.97,ha='left',fontsize=23,fontweight='medium',color='#294d3b')
fig.text(.06,.91,'Ingombri nominali e percorso del carico · millimetri · obiettivo 20 kg utili',fontsize=13,va='top')
ax=fig.add_axes([.05,.16,.45,.70]);side=fig.add_axes([.53,.39,.43,.42])
for a in (ax,side):
    a.set_aspect('equal');a.axis('off')
def rect(a,x,y,w,h,c,ec='#45504b',radius=0,**kwargs):
    p=FancyBboxPatch((x,y),w,h,boxstyle=f'round,pad=0,rounding_size={radius}',facecolor=c,edgecolor=ec,linewidth=.9,**kwargs) if radius else Rectangle((x,y),w,h,facecolor=c,edgecolor=ec,linewidth=.9,**kwargs)
    a.add_patch(p)
def dim(a,p1,p2,text,offset=(0,0)):
    a.annotate('',xy=p1,xytext=p2,arrowprops={'arrowstyle':'<->','lw':.8,'color':'#535b57'})
    a.text((p1[0]+p2[0])/2+offset[0],(p1[1]+p2[1])/2+offset[1],text,ha='center',va='center',bbox={'facecolor':'white','edgecolor':'none','pad':2})
rect(ax,-150,-160,300,320,'#e6e9e2',radius=18)
for x in (-132,132):
    for y in (-142,142):ax.add_patch(Circle((x,y),15,fill=False,edgecolor='#68766e',linestyle='--',linewidth=.8))
rect(ax,-140,-50,280,200,'#f7f7f1',radius=20)
for x in (-113,113):
    for y in (-24,50,124):ax.add_patch(Circle((x,y),3,facecolor='#afb8b1',edgecolor='#637268',linewidth=.5))
rect(ax,-118,35,130,30,'none',ec='#739185',linestyle='--')
ax.text(0,91,'Piatto plastico',ha='center',fontsize=14)
ax.text(0,71,'280 × 200 × 4',ha='center')
ax.text(-53,26,'Cella sotto il piatto',ha='center',fontsize=10,color='#547160')
rect(ax,-138,-141,142,58,'#345844',radius=5)
rect(ax,-107,-123,80,22,'#173329')
ax.text(-67,-112,'OLED posseduto',color='#d8ead4',ha='center',va='center',fontsize=9)
for y in (-129,-98):
    rect(ax,42,y-10,70,20,'#d3dfd2',radius=1)
    for j in range(1,4):ax.plot([42+j*17.5]*2,[y-10,y+10],color='#78937f',lw=.7)
ax.text(77,-151,'8 tasti',ha='center',fontsize=10)
rect(ax,150,92,12,20,'#596f61')
ax.annotate('USB-C',xy=(162,102),xytext=(173,125),fontsize=10,arrowprops={'arrowstyle':'-','lw':.6})
dim(ax,(-150,-188),(150,-188),'300')
dim(ax,(-178,-160),(-178,160),'320',offset=(-1,0))
ax.text(0,-216,'VISTA DALL’ALTO  ·  fronte in basso',ha='center',fontsize=10,color='#547160')
ax.set_xlim(-205,209);ax.set_ylim(-224,183)

rect(side,-150,13,300,6,'#aeb8b3')
rect(side,-118,25,130,22,'#bacbc2')
rect(side,-118,19,25,6,'#95ada0')
rect(side,-13,47,25,22,'#95ada0')
rect(side,-138,69,276,8,'#b7c0bb')
rect(side,-140,77,280,4,'#eceee6')
for x in (-132,132):
    rect(side,x-15,0,30,13,'#4b5c52',radius=2)
    side.plot([x,x],[13,36],color='#68796f',lw=2)
for x in (-112,112):side.plot([x,x],[7.5,67.5],color='#8b9690',lw=1.2,linestyle='--')
side.annotate('Carico',xy=(40,81),xytext=(40,106),ha='center',arrowprops={'arrowstyle':'-|>','color':'#294d3b','lw':1.6},color='#294d3b')
side.annotate('Distanziale mobile 22',xy=(0,59),xytext=(50,52),fontsize=10,arrowprops={'arrowstyle':'-','lw':.6})
side.annotate('L6D 30 kg',xy=(-52,37),xytext=(22,30),fontsize=10,arrowprops={'arrowstyle':'-','lw':.6})
side.annotate('Distanziale fisso 6',xy=(-105,22),xytext=(-142,-20),fontsize=10,arrowprops={'arrowstyle':'-','lw':.6})
dim(side,(164,0),(164,81),'81',offset=(7,0))
side.text(0,124,'SCHEMA X / Z  ·  gruppo di pesatura',ha='center',fontsize=11,color='#547160')
side.set_xlim(-155,190);side.set_ylim(-28,138)

fig.text(.54,.31,'Piatto plastico → alluminio 8 → cella → alluminio 6 → piedi',fontsize=11,color='#294d3b',va='top')
fig.text(.54,.275,'Fondo piano per vassoi e schede. Guscio separato dal piatto.\nPiedi M6 regolabili, appoggi D30 agli angoli.\nArresti regolabili: gioco nominale 1,5 mm da caratterizzare.',fontsize=11,linespacing=1.7,va='top')
fig.text(.06,.065,'Schema di disposizione, non tavola esecutiva. Per fori, filetti e svasature usare STEP e manufacturing-notes.md.\nPortata, giochi stampati e attacchi del display richiedono verifica del prototipo.',fontsize=10,color='#627168',linespacing=1.7)
fig.savefig(OUT/'layout-and-load-path.svg',facecolor='white')
fig.savefig(OUT/'layout-and-load-path.png',dpi=150,facecolor='white')
print(OUT/'layout-and-load-path.png')
