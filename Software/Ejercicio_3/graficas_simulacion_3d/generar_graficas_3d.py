"""Uso: python generar_graficas_3d.py resultados_simulacion_3d.csv carpeta_salida
Requiere numpy y matplotlib. No suaviza ni descarta muestras del CSV.
"""
import sys, csv, json, shutil, zipfile, io
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

src=Path(sys.argv[1]); out=Path(sys.argv[2]); out.mkdir(parents=True,exist_ok=True)
d=np.genfromtxt(src,delimiter=',',names=True)
t=d['tiempo_s']; names=d.dtype.names
assert len(d)>1 and np.all(np.diff(t)>0)
assert all(np.all(np.isfinite(d[k])) for k in names)
assert all(np.all(d[k]>=0) for k in names if k.startswith('sigma_'))
on=np.flatnonzero(d['empuje_n']>0); ignition=t[on[0]]; cutoff=t[on[-1]+1]
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.titlesize':12,
 'axes.labelsize':10,'figure.titlesize':15,'axes.spines.top':False,'axes.spines.right':False,
 'axes.grid':True,'grid.alpha':.22,'lines.linewidth':1.3,'savefig.dpi':250,
 'pdf.fonttype':42,'ps.fonttype':42})
blue='#146A9E'; orange='#D56C28'; green='#2F876D'; colors=[blue,orange,green]
pdf=PdfPages(out/'graficas_simulacion_3d.pdf'); manifest=[]
def save(fig,name,title):
    fig.suptitle(title)
    buf=io.BytesIO()
    fig.savefig(buf,format='png',bbox_inches='tight')
    (out/(name+'.png')).write_bytes(buf.getvalue())
    fig.savefig(out/(name+'.pdf'),bbox_inches='tight')
    pdf.savefig(fig,bbox_inches='tight'); plt.close(fig); manifest.append((name,title))
def panels(n=3):
    fig,ax=plt.subplots(n,1,figsize=(9,2.1*n+1),sharex=True,layout='constrained')
    ax=np.atleast_1d(ax)
    for a in ax:
        a.axvline(ignition,color='#666666',ls=':',lw=1)
        a.axvline(cutoff,color='#666666',ls='--',lw=1)
        a.set_xlim(t[0],t[-1]); a.ticklabel_format(axis='y',style='plain',useOffset=False)
    ax[-1].set_xlabel('Tiempo desde el inicio de la simulación (s)')
    fig.supxlabel('Líneas verticales: despegue (punteada) y apagado del motor (discontinua)',fontsize=8,color='#555555')
    return fig,ax
def pair(ax,ref,est,label,scale=1):
    ax.plot(t,d[ref]*scale,color=blue,label='Referencia')
    ax.plot(t,d[est]*scale,color=orange,ls='--',label='Estimación')
    ax.set_ylabel(label)
def errorset(num,name,title,keys,labels,scale=1):
    f,ax=panels(len(keys))
    for a,k,lab in zip(ax,keys,labels):
        s=3*d['sigma_'+k]*scale
        a.fill_between(t,-s,s,color=blue,alpha=.12,label=r'Banda $\pm3\sigma$')
        a.plot(t,s,color=blue,lw=.65); a.plot(t,-s,color=blue,lw=.65)
        a.axhline(0,color='#888888',lw=.6)
        a.plot(t,d['error_'+k]*scale,color=orange,label=('Error angular local' if k.startswith('theta_') else 'Error: real − estimado'))
        a.set_ylabel(lab)
    ax[0].legend(loc='upper right',fontsize=9)
    save(f,f'{num:02d}_{name}',title)

f=plt.figure(figsize=(11,7),layout='constrained')
a=f.add_subplot(121,projection='3d'); b=f.add_subplot(122)
for typ,col,ls,label in [('ref',blue,'-','Referencia'),('est',orange,'--','Estimación')]:
    a.plot(d[f'p_n_{typ}_m']/1000,d[f'p_e_{typ}_m']/1000,-d[f'p_d_{typ}_m']/1000,color=col,ls=ls,label=label)
    b.plot(d[f'p_e_{typ}_m'],d[f'p_n_{typ}_m'],color=col,ls=ls,label=label)
a.set(xlabel='Norte (km)',ylabel='Este (km)',zlabel='Altura (km)')
a.set_box_aspect((1,1,2)); a.view_init(elev=20,azim=-60)
a.set_title('Vista 3D (escalas distintas por eje)',fontsize=10)
b.set(xlabel='Este (m)',ylabel='Norte (m)',title='Proyección horizontal'); b.set_aspect('equal',adjustable='box'); b.legend()
save(f,'01_trayectoria_3d','Trayectoria de referencia y estimación')

for num,prefix,title,unit in [(2,'p','Posición en NED','m'),(3,'v','Velocidad en NED','m/s')]:
    f,ax=panels()
    suffix='m' if prefix=='p' else 'm_s'
    for a,e,lab in zip(ax,'ned',['Norte','Este','Abajo']): pair(a,f'{prefix}_{e}_ref_{suffix}',f'{prefix}_{e}_est_{suffix}',f'{lab} ({unit})')
    ax[0].legend(); save(f,f'{num:02d}_{prefix}_ned',title)
f,ax=panels(2)
pair(ax[0],'altura_ref_m','altura_est_m','Altura (km)',.001)
pair(ax[1],'velocidad_ref_m_s','velocidad_est_m_s','Velocidad de ascenso (m/s)')
ax[0].legend();save(f,'04_movimiento_vertical','Altura y velocidad vertical')
errorset(5,'error_posicion','Errores de posición e incertidumbre',[f'p_{e}_m' for e in 'ned'],[f'{e} (m)' for e in ['Norte','Este','Abajo']])
errorset(6,'error_velocidad','Errores de velocidad e incertidumbre',[f'v_{e}_m_s' for e in 'ned'],[f'{e} (m/s)' for e in ['Norte','Este','Abajo']])

# q y -q representan la misma orientación. Alinear signos solo para dibujar.
qr=np.column_stack([d[f'q{i}_ref'] for i in range(4)])
qe=np.column_stack([d[f'q{i}_est'] for i in range(4)])
qe*=np.where(np.sum(qr*qe,axis=1)<0,-1,1)[:,None]
f,ax=panels(4)
for i,a in enumerate(ax):
    a.plot(t,qr[:,i],color=blue,label='Referencia');a.plot(t,qe[:,i],color=orange,ls='--',label='Estimación');a.set_ylabel(f'$q_{i}$')
ax[0].legend();save(f,'07_cuaterniones','Orientación: componentes del cuaternión cuerpo → NED')
f,ax=panels(2)
ax[0].plot(t,d['error_orientacion_deg'],color=orange); ax[0].set_ylabel('Error angular total (°)')
incl=np.rad2deg(np.arccos(np.clip(-(2*(qe[:,1]*qe[:,3]-qe[:,0]*qe[:,2])),-1,1)))
ax[1].plot(t,d['inclinacion_ref_deg'],color=blue,label='Referencia'); ax[1].plot(t,incl,color=orange,ls='--',label='Estimación');ax[1].set_ylabel('Inclinación respecto\na la vertical (°)');ax[1].legend()
save(f,'08_orientacion','Error angular total e inclinación del eje longitudinal')
errorset(9,'error_angular','Errores angulares locales e incertidumbre',[f'theta_{e}_rad' for e in 'xyz'],[rf'$\delta	heta_{e}$ (°)' for e in 'xyz'],180/np.pi)
for num,sensor,title,unit in [(10,'acc','Sesgos del acelerómetro','m/s²'),(11,'giro','Sesgos del giróscopo','rad/s')]:
    f,ax=panels(); suffix='m_s2' if sensor=='acc' else 'rad_s'
    for a,e in zip(ax,'xyz'): pair(a,f'sesgo_{sensor}_{e}_real_{suffix}',f'sesgo_{sensor}_{e}_{suffix}',f'Eje {e.upper()} ({unit})')
    ax[0].legend();save(f,f'{num:02d}_sesgos_{sensor}',title)
errorset(12,'error_sesgos_acc','Errores de los sesgos del acelerómetro',[f'ba_{e}_m_s2' for e in 'xyz'],[f'Eje {e.upper()} (m/s²)' for e in 'xyz'])
errorset(13,'error_sesgos_giro','Errores de los sesgos del giróscopo',[f'bg_{e}_rad_s' for e in 'xyz'],[f'Eje {e.upper()} (rad/s)' for e in 'xyz'])
f,ax=panels(5)
for a,(prefix,suffix,lab,axes,scale) in zip(ax,[('p','m','Posición (m)','ned',1),('v','m_s','Velocidad (m/s)','ned',1),('theta','rad','Orientación (°)','xyz',180/np.pi),('ba','m_s2','Sesgo acc. (m/s²)','xyz',1),('bg','rad_s','Sesgo giro (rad/s)','xyz',1)]):
    for col,e in zip(colors,axes):a.plot(t,d[f'sigma_{prefix}_{e}_{suffix}']*scale,color=col,label=e.upper())
    a.set_ylabel(lab);a.set_yscale('log');a.legend(loc='upper right',ncol=3)
save(f,'14_incertidumbres','Desviaciones típicas del estado de error (escala logarítmica)')
f,ax=panels(4)
for a,k,lab in zip(ax,['masa_kg','empuje_n','rapidez_ref_m_s','inclinacion_ref_deg'],['Masa (kg)','Empuje (N)','Rapidez (m/s)','Inclinación (°)']):
    a.plot(t,d[k],color=blue);a.set_ylabel(lab)
save(f,'15_perfil_vuelo','Perfil del vuelo simulado')
ep=np.sqrt(sum(d[f'error_p_{e}_m']**2 for e in 'ned'))
ev=np.sqrt(sum(d[f'error_v_{e}_m_s']**2 for e in 'ned'))
f,ax=panels(2);ax[0].plot(t,ep,color=blue);ax[0].set_ylabel('Norma del error\nde posición (m)');ax[1].plot(t,ev,color=orange);ax[1].set_ylabel('Norma del error\nde velocidad (m/s)')
save(f,'16_errores_3d','Magnitud de los errores de posición y velocidad')
pdf.close()
with open(out/'metricas.csv','w',newline='') as f:
    w=csv.writer(f);w.writerow(['magnitud','RMSE','maximo_absoluto','valor_final','unidad'])
    for lab,v,unit in [('error_posicion_3d',ep,'m'),('error_velocidad_3d',ev,'m/s'),('error_angular',d['error_orientacion_deg'],'grados')]:
        w.writerow([lab,np.sqrt(np.mean(v[1:]**2)),np.max(np.abs(v)),v[-1],unit])
    for k in names:
        if k.startswith('error_') and k!='error_orientacion_deg':
            v=d[k];w.writerow([k,np.sqrt(np.mean(v[1:]**2)),np.max(np.abs(v)),v[-1],'segun_sufijo'])
readme='''GRÁFICAS DE LA SIMULACIÓN 3D
Fuente: resultados_simulacion_3d.csv adjuntado por el usuario.
Todas las muestras se representan sin suavizado. PNG a 250 dpi y PDF vectorial.
Las escalas de la vista 3D son diferentes; la proyección horizontal tiene escala igual.
Posición y velocidad: sistema NED; altura = -pD y ascenso = -vD.
Errores aditivos, altura y ascenso: referencia - estimacion.
Error angular: log(conjugado(q_est) * q_ref), expresado en el cuerpo estimado.
Bandas: ±3 veces la desviación típica de la diagonal de P. Son bandas marginales
del modelo, no una garantía de cobertura ni una prueba Monte Carlo.
Cuaterniones estimados alineados de signo con la referencia solo al dibujar.
La inclinación estimada se calcula desde la dirección del eje X del cuerpo.
Líneas verticales: despegue y apagado, detectados en la columna de empuje.
Las métricas RMS excluyen la fila inicial, igual que la salida del simulador.
Los límites de los ejes no se recortan para ocultar transitorios.

FIGURAS
'''+''.join(f'{n}: {title}\n' for n,title in manifest)
(out/'LEEME.txt').write_text(readme,encoding='utf-8')
shutil.copy2(Path(__file__),out/'generar_graficas_3d.py')
shutil.copy2(src,out/'resultados_simulacion_3d.csv')
with zipfile.ZipFile(out.parent/'graficas_simulacion_3d.zip','w',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(out.iterdir()):z.write(p,'graficas_simulacion_3d/'+p.name)
# Contact sheet for visual review, not a delivered figure.
from PIL import Image,ImageOps,ImageDraw
sheet=Image.new('RGB',(1600,4*450),'white')
for i,(name,title) in enumerate(manifest):
    im=Image.open(out/(name+'.png'));im.thumbnail((390,420))
    x=(i%4)*400;y=(i//4)*450;sheet.paste(im,(x+(400-im.width)//2,y));ImageDraw.Draw(sheet).text((x+8,y+425),name,fill='black')
sheet.save('/tmp/graficas3d_contacto.jpg')
print(json.dumps({'filas':len(d),'figuras':len(manifest),'t_final':float(t[-1]),'altura_final':float(d['altura_ref_m'][-1]),'rmse_pos':float(np.sqrt(np.mean(ep[1:]**2))),'rmse_angular':float(np.sqrt(np.mean(d['error_orientacion_deg'][1:]**2)))},indent=2))
