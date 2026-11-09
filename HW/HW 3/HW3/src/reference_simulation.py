"""Numerical reference for HW3 articulated spring--mass runner.
Python 3 + numpy + matplotlib. Reproduces report figures and statistics.
Conventions exactly match Assignment 3 Eqs. (1)--(5).
"""
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
import json
OUT = Path(__file__).resolve().parent.parent/'figures'; OUT.mkdir(exist_ok=True)

P=dict(m=70.,g=9.81,k=20000.,l0=1.,d=.2,J=3.5,aoa=np.deg2rad(68.))

def force(y,fp,mode,h,Kp,Kd,D):
    x,z,vx,vz,ph,w=y
    rx=x+P['d']*np.sin(ph)-fp
    rz=z-P['d']*np.cos(ph)
    ell=np.hypot(rx,rz)
    Fs=P['k']*max(0.,P['l0']-ell)
    if Fs <= 0:
        return (0.,0.,0.,0.,ell)
    if mode=='vpp':
        L=P['d']+h
        q=ell*ell+L*(rz*np.cos(ph)-rx*np.sin(ph))
        if abs(q)<1e-9:
            raise ValueError('VPP singular geometry: q near zero')
        tau=-Fs*ell*L*(rx*np.cos(ph)+rz*np.sin(ph))/q -D*(Fs/P['m'])*w
    elif mode=='pd':
        tau=-(Kp*ph+Kd*w)*(Fs/P['m'])
    else:
        raise ValueError(mode)
    Fx=rx*Fs/ell+rz*tau/ell**2
    Fz=rz*Fs/ell-rx*tau/ell**2
    return Fx,Fz,tau,Fs,ell

def run(mode,h=.2,Kp=200,Kd=30,D=0,T=4,dt=1e-4,phi0=-.02):
    # Original course file begins with hip height 1.1m. The CoM is 0.2m above hip.
    y=np.array([0.,1.1+P['d']*np.cos(phi0),5.,0.,phi0,0.])
    stance=False; fp=0.;prev_hip_z=y[1]-P['d']*np.cos(phi0)
    rows=[]; events=[]; td_time=-100
    for j in range(int(T/dt)):
        t=j*dt
        x,z,vx,vz,phi,w=y
        hz=z-P['d']*np.cos(phi); hx=x+P['d']*np.sin(phi)
        if not stance and hz <= P['l0']*np.sin(P['aoa']) and prev_hip_z>P['l0']*np.sin(P['aoa']) and (vz+P['d']*np.sin(phi)*w)<0:
            fp=hx+P['l0']*np.cos(P['aoa']);stance=True;td_time=t;events.append([t,'touchdown',fp])
        if stance:
            rx=hx-fp;rz=hz;ell=np.hypot(rx,rz)
            ell_dot=(rx*(vx+P['d']*np.cos(phi)*w)+rz*(vz+P['d']*np.sin(phi)*w))/ell
            if ell>=P['l0'] and ell_dot>0 and t-td_time>0.03:
                stance=False;events.append([t,'liftoff',fp])
        def rhs(s):
            if stance:
                Fx,Fz,tau,Fs,_=force(s,fp,mode,h,Kp,Kd,D)
            else: Fx=Fz=tau=Fs=0.
            ph=s[4]
            return np.array([s[2],s[3],Fx/P['m'],Fz/P['m']-P['g'],s[5],(tau+P['d']*(Fx*np.cos(ph)+Fz*np.sin(ph)))/P['J']])
        if stance: Fx,Fz,tau,Fs,_=force(y,fp,mode,h,Kp,Kd,D)
        else: Fx=Fz=tau=Fs=0.
        rows.append([t,*y,float(stance),fp,Fx,Fz,tau,Fs])
        a=rhs(y);b=rhs(y+dt*a/2);c=rhs(y+dt*b/2);e=rhs(y+dt*c)
        y=y+dt*(a+2*b+2*c+e)/6.
        prev_hip_z=hz
        if not np.all(np.isfinite(y)) or abs(y[4])>np.pi or y[1]<0.6:
            break
    return np.array(rows),events

def body_lines(data):
    # columns: t x z vx vz phi w stance fp Fx Fz tau Fs
    b=data[(data[:,7]>.5)&(data[:,12]>100.)]
    t,x,z,vx,vz,phi,w,stance,fp,Fx,Fz,tau,Fs=b.T
    xb=(fp-x)*np.cos(phi)-z*np.sin(phi)
    zb=-(fp-x)*np.sin(phi)-z*np.cos(phi)
    fxb=Fx*np.cos(phi)+Fz*np.sin(phi)
    fzb=-Fx*np.sin(phi)+Fz*np.cos(phi)
    norms=np.hypot(fxb,fzb)
    lines=np.column_stack([xb,zb,fxb/norms,fzb/norms])
    return lines,b

def fit(data,h=.2):
    L,b=body_lines(data)
    A=np.column_stack([-L[:,3],L[:,2]])
    rhs=(A*L[:,:2]).sum(axis=1)
    p=np.linalg.lstsq(A,rhs,rcond=None)[0]
    rms=float(np.sqrt(np.mean((A@p-rhs)**2)))
    target=float(np.sqrt(np.mean((A@np.array([0.,h])-rhs)**2)))
    return dict(n=int(len(L)),p_x=float(p[0]),p_z=float(p[1]),rms_m=rms,target_rms_m=target,td_count=None)

def save_plot():
    v,e_v=run('vpp');q,e_q=run('pd')
    vf=fit(v);pf=fit(q)
    vf['td_count']=sum(e[1]=='touchdown' for e in e_v)
    pf['td_count']=sum(e[1]=='touchdown' for e in e_q)
    print('VPP',vf,'end time',v[-1,0]);print('PD',pf,'end time',q[-1,0]);print('VPP EVENTS',e_v[:5]);print('PD EVENTS',e_q[:5])
    with open(OUT/'results.json','w') as f:json.dump(dict(vpp=vf,pd=pf,parameters=dict(P,h=.2,Kp=200,Kd=30,D=0,dt=1e-4,phi0=-.02)),f,indent=2)
    # Lines: select a single representative stance around 0.6-0.9s if exists
    fig,axs=plt.subplots(1,2,figsize=(10.5,4.9),sharex=True,sharey=True)
    for ax,mode,a,info in [(axs[0],'Geometric VPP',v,vf),(axs[1],'Load-modulated PD',q,pf)]:
        L,b=body_lines(a)
        # Use samples from time 0.5--2.0, evenly spaced across all stances
        mask=(b[:,0]>=0.5)&(b[:,0]<=min(2.3,a[-1,0]))
        L=L[mask]
        if len(L)>20:
            sel=np.linspace(0,len(L)-1,18).astype(int)
            L=L[sel]
        for x0,z0,ux,uz in L:
            # Draw GRF *line of action* (both directions), not just an arrow
            lam=np.linspace(-0.6,1.8,2)
            ax.plot(x0+lam*ux,z0+lam*uz,alpha=.48,lw=1.1,color='#2874A6' if mode=='Geometric VPP' else '#D97721')
            ax.plot(x0,z0,'.',color='gray',ms=2)
        ax.plot(0,0,'ko',ms=5,label='CoM')
        ax.plot(0,.2,'r*',ms=10,label='Commanded VPP')
        ax.plot(info['p_x'],info['p_z'],'x',ms=10,mew=2,color='#177245',label='Best-fit point')
        ax.axvline(0,color='#666666',ls=':',lw=.8)
        ax.axhline(0,color='#666666',ls=':',lw=.8)
        ax.set_title(f'{mode}\nfit RMS {info["rms_m"]*100:.2f} cm')
        ax.set(xlabel='Body-frame horizontal position (m)', xlim=(-.8,.8),ylim=(-1.6,.75))
        ax.grid(alpha=.15)
    axs[0].set_ylabel('Body-frame vertical position (m)')
    h0,l0=axs[1].get_legend_handles_labels();fig.legend(h0,l0,loc='lower center',ncol=3,fontsize=9,bbox_to_anchor=(.5,-.025))
    fig.tight_layout(rect=(0,.06,1,1))
    fig.savefig(OUT/'grf_pivots.pdf',bbox_inches='tight');plt.close(fig)
    # Pitch traces comparison
    vd,ed=run('vpp',D=10,T=4)
    fig,ax=plt.subplots(figsize=(10,3.55))
    for a,label,col in [(v,'VPP, no damping','#2874A6'),(vd,'VPP + load-modulated pitch damping','#177245'),(q,'Load-modulated PD','#D97721')]:
        ax.plot(a[:,0],np.rad2deg(a[:,5]),label=label,color=col,lw=1.5)
    ax.axhline(0,color='k',lw=.7)
    ax.set(xlabel='Time (s)',ylabel='Trunk pitch (deg)',xlim=(0,3.5),ylim=(-60,60))
    ax.grid(alpha=.15);ax.legend(fontsize=9,loc='lower right')
    fig.tight_layout();fig.savefig(OUT/'pitch_comparison.pdf');plt.close(fig)
    print('DAMPED VPP',fit(vd),'end',vd[-1,0]);
    # sensitivity for various h (not on too much horizon)
    fig,ax=plt.subplots(figsize=(10,3.45));sens={}
    for hh in [-.2,-.1,0.,.1,.2,.4]:
        w,ee=run('vpp',h=hh,T=2.2)
        sens[str(hh)]=dict(endtime=float(w[-1,0]),max_abs_pitch_degrees=float(np.max(np.abs(np.rad2deg(w[:,5])))))
        ax.plot(w[:,0],np.rad2deg(w[:,5]),label=f'h={hh:+.1f} m',lw=1.2)
    ax.set(xlabel='Time (s)',ylabel='Trunk pitch (deg)',xlim=(0,2.2),ylim=(-90,90));ax.grid(alpha=.15);ax.legend(ncol=3,fontsize=9,loc='lower left')
    fig.tight_layout();fig.savefig(OUT/'vpp_height_sensitivity.pdf');plt.close(fig)
    with open(OUT/'height_sensitivity.json','w') as f:json.dump(sens,f,indent=2)
    print('sens',sens)

if __name__=='__main__':save_plot()
