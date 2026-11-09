function run_assignment3()
% R16868 Assignment 3: articulated SLIP running with VPP and loaded PD.
% Pure MATLAB implementation; requires no Simulink or other toolboxes.
% Original m08.slx and m08_params.m are included as course_starter files.
% Fixed-step RK4 with touchdown and liftoff events and a single stance leg.
% Run this function from anywhere: addpath('src'); run_assignment3
root = fileparts(fileparts(mfilename('fullpath')));
figdir = fullfile(root, 'figures');
if ~exist(figdir,'dir'), mkdir(figdir); end
p = struct('m',70,'g',9.81,'k',20000,'l0',1,'d',0.2,'J',3.5,...
           'aoa',68*pi/180,'h',0.2,'Kp',200,'Kd',30,'D',0,...
           'dt',1e-4,'T',4,'phi0',-0.02);

vpp = simulate_runner(p,'vpp');
pd = simulate_runner(p,'pd');
vfit = pivot_fit(vpp,p.h);
pfit = pivot_fit(pd,p.h);
fprintf('VPP: best fit (%.6f, %.6f) m, RMS %.6e m (%d points)\n', ...
    vfit.xy(1), vfit.xy(2), vfit.rms, vfit.n);
fprintf('PD:  best fit (%.6f, %.6f) m, RMS %.6f m (%d points)\n', ...
    pfit.xy(1), pfit.xy(2), pfit.rms, pfit.n);

f = figure('Color','w','Position',[100 100 1100 460]);
subplot(1,2,1); plot_lines(vpp,vfit,p.h,'Geometric VPP');
subplot(1,2,2); plot_lines(pd,pfit,p.h,'Load-modulated PD');
print(f,fullfile(figdir,'matlab_grf_pivots.pdf'),'-dpdf','-bestfit');close(f);

p.D=10; damped=simulate_runner(p,'vpp');p.D=0;
dfit=pivot_fit(damped,p.h);
fprintf('Damped VPP: best fit (%.6f, %.6f) m, RMS %.6f m\n', ...
    dfit.xy(1),dfit.xy(2),dfit.rms);
f=figure('Color','w','Position',[100 100 1000 370]);
plot(vpp(:,1),vpp(:,6)*180/pi,'LineWidth',1.1);hold on;
plot(damped(:,1),damped(:,6)*180/pi,'LineWidth',1.1);
plot(pd(:,1),pd(:,6)*180/pi,'LineWidth',1.1);
ylim([-60 60]);xlim([0 3.5]);grid on;
xlabel('Time (s)');ylabel('Trunk pitch (deg)');
legend('VPP','VPP + load-modulated damping','Load-modulated PD','Location','best');
print(f,fullfile(figdir,'matlab_pitch_comparison.pdf'),'-dpdf','-bestfit');close(f);

f=figure('Color','w','Position',[100 100 1000 360]);
for hh=[-0.2,-0.1,0,0.1,0.2,0.4]
    p.h=hh;p.T=2.2;
    s=simulate_runner(p,'vpp');
    plot(s(:,1),s(:,6)*180/pi,'LineWidth',1.1, ...
        'DisplayName',sprintf('h=%+.1f m',hh));hold on;
end
ylim([-90 90]);xlim([0 2.2]);grid on;
xlabel('Time (s)');ylabel('Trunk pitch (deg)');legend('Location','best');
print(f,fullfile(figdir,'matlab_vpp_height_sensitivity.pdf'),'-dpdf','-bestfit');close(f);
end

function rows = simulate_runner(p,mode)
% cols: time, x,z,vx,vz,phi,omega,stance,fp,Fx,Fz,tau,Fs
n = ceil(p.T/p.dt);
rows = zeros(n,13);
y = [0;1.1+p.d*cos(p.phi0);5;0;p.phi0;0];
stance = false;fp=0;
prev_hz = y(2)-p.d*cos(y(5)); tD=-1e3;
for i=1:n
    t=(i-1)*p.dt;
    x=y(1);z=y(2);vx=y(3);vz=y(4);phi=y(5);w=y(6);
    hx=x+p.d*sin(phi);hz=z-p.d*cos(phi);
    if ~stance && hz<=p.l0*sin(p.aoa) && prev_hz>p.l0*sin(p.aoa) ...
           && (vz+p.d*sin(phi)*w)<0
        stance=true;fp=hx+p.l0*cos(p.aoa);tD=t;
    end
    if stance
        rx=hx-fp;rz=hz;ell=hypot(rx,rz);
        ell_dot=(rx*(vx+p.d*cos(phi)*w)+rz*(vz+p.d*sin(phi)*w))/ell;
        if ell>=p.l0 && ell_dot>0 && t-tD>0.03
            stance=false;
        end
    end
    if stance
        [Fx,Fz,tau,Fs] = leg_force(y,fp,p,mode);
    else
        Fx=0;Fz=0;tau=0;Fs=0;
    end
    rows(i,:)=[t y' double(stance) fp Fx Fz tau Fs];
    a=dynamics(y,stance,fp,p,mode);
    b=dynamics(y+p.dt*a/2,stance,fp,p,mode);
    c=dynamics(y+p.dt*b/2,stance,fp,p,mode);
    e=dynamics(y+p.dt*c,stance,fp,p,mode);
    y=y+p.dt*(a+2*b+2*c+e)/6;
    prev_hz=hz;
    if any(~isfinite(y)) || abs(y(5))>pi || y(2)<0.6
        rows=rows(1:i,:);
        warning('Terminated %s, h=%.2f at %.3f s: fall/pitch limit',mode,p.h,t);
        break;
    end
end
end

function dy = dynamics(y,stance,fp,p,mode)
if stance
    [Fx,Fz,tau]=leg_force(y,fp,p,mode);
else
    Fx=0;Fz=0;tau=0;
end
ph=y(5);
dy=[y(3);y(4);Fx/p.m;Fz/p.m-p.g;y(6); ...
    (tau+p.d*(Fx*cos(ph)+Fz*sin(ph)))/p.J];
end

function [Fx,Fz,tau,Fs,ell]=leg_force(y,fp,p,mode)
rx=y(1)+p.d*sin(y(5))-fp;
rz=y(2)-p.d*cos(y(5));
ell=hypot(rx,rz);
Fs=p.k*max(0,p.l0-ell);
if Fs<=0
    Fx=0;Fz=0;tau=0;return;
end
phi=y(5);omega=y(6);
if strcmp(mode,'vpp')
    L=p.d+p.h;
    denom=ell^2+L*(rz*cos(phi)-rx*sin(phi));
    if abs(denom)<1e-9,error('Singular VPP lever arm');end
    tau=-Fs*ell*L*(rx*cos(phi)+rz*sin(phi))/denom ...
        -p.D*(Fs/p.m)*omega;
elseif strcmp(mode,'pd')
    tau=-(p.Kp*phi+p.Kd*omega)*(Fs/p.m);
else
    error('Unknown controller');
end
Fx=rx*Fs/ell+rz*tau/(ell^2);
Fz=rz*Fs/ell-rx*tau/(ell^2);
end

function [lines,times]=body_lines(data)
mask=data(:,8)>0.5 & data(:,13)>100;
a=data(mask,:);t=a(:,1);
x=a(:,2);z=a(:,3);phi=a(:,6);fp=a(:,9);
fx=a(:,10);fz=a(:,11);
xb=(fp-x).*cos(phi)-z.*sin(phi);
zb=-(fp-x).*sin(phi)-z.*cos(phi);
fxb=fx.*cos(phi)+fz.*sin(phi);
fzb=-fx.*sin(phi)+fz.*cos(phi);
mag=hypot(fxb,fzb);
lines=[xb zb fxb./mag fzb./mag];times=t;
end

function result=pivot_fit(data,h)
[L,~]=body_lines(data);
A=[-L(:,4) L(:,3)];
b=sum(A.*L(:,1:2),2);
xy=A\b;
rms=sqrt(mean((A*xy-b).^2));
target_rms=sqrt(mean((A*[0;h]-b).^2));
result=struct('xy',xy,'rms',rms,'target_rms',target_rms,'n',size(L,1));
end

function plot_lines(data,fit,h,caption)
[L,t]=body_lines(data);
L=L(t>=0.5 & t<=2.3,:);
idx=unique(round(linspace(1,size(L,1),18)));
for j=idx
    x0=L(j,1);z0=L(j,2);ux=L(j,3);uz=L(j,4);
    ls=linspace(-0.6,1.8,2);
    plot(x0+ls*ux,z0+ls*uz,'LineWidth',0.8);hold on;
end
plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',5);
plot(0,h,'rp','MarkerFaceColor','r','MarkerSize',10);
plot(fit.xy(1),fit.xy(2),'gx','LineWidth',2,'MarkerSize',10);
xline(0,':');yline(0,':');axis equal;xlim([-0.8 0.8]);ylim([-1.6 0.75]);grid on;
title(sprintf('%s (RMS %.2f cm)',caption,100*fit.rms));
xlabel('Body x (m)');ylabel('Body z (m)');
end
