%% SOC PID benchmark configuration file for Optimal randomness search
%Globalized Constrained Nelder Mead GCNM parameters
GCNMEnable=1;
optimPeriod=300;        %optimization execution period (s)
resetThreshold=0.1;     %probabilistic restart threshold
refSignalPeriod=300;    %Reference signal period
refAmplitude=1;        %reference amplitude
%constraints limits
OVLim=2;             %5% overshoot
TSLim=70;               %Settling time limit (s)
L=1;                  %delay value L=0.1,1,10;
N=100;                  %derivative filter N (Zero for PI controller)

%initial condition PI control
kpo=1.2;  %10  0.06
kio=0.6; %3    0.05
%optimization gains

% PI controller gains limits

if L<1 %big searching space(Kp,,ki>=100 for L<0.1)
    kpo=10;  %10  0.06
    kio=3; %3    0.05
    maxKp=10; minKp=0.01;
    maxKi=3; minKi=0.001;
elseif L>=1 && L<10 %average searching space (kp,ki<1 for L>10)
    kpo=1.2;  %10  0.06
    kio=0.6; %3    0.05
    maxKp=1; minKp=0.01;
    maxKi=0.5; minKi=0.001;
elseif L>=10 %small searching space (kp,ki<1 for L>10)
    kpo=0.06;  %10  0.06
    kio=0.05; %3    0.05
    maxKp=0.1; minKp=0.01;
    maxKi=0.1; minKi=0.001;
end





%% Run the Benchmark file
sim("SOCPIControlBenchmarkReal.slx")

% SOC PI time response
benchmarkFiguresSOC

%% SOC performance indices
SOCConvTime= timeSub(  converTimeInv1s(costJSub,10,1e-3)); %convergence time SOC
OVFinal=OVOptim(end);   %Final OV
TSFinal=TSOptim(end);   %Final Settling time
RMSFinal=rms(yOut);             %RMS value of the system output
ISEFinal=     (1/length(yOut))*sum((yOut-ref).^2);
IAEFinal=     (1/length(yOut))*sum(abs(yOut));
ITAEFinal=    (1/length(yOut))*sum(time.*(yOut-ref).^2);
RMSE=    sqrt(  ((1/length(yOut)))*sum(yOut-ref).^2);
varNames={'SOC convergence time','OV','Settling Time','RMS','ISE','IAE','RMSE'};
table(SOCConvTime,OVFinal,TSFinal,RMSFinal,ISEFinal,IAEFinal,RMSE,...
    'VariableNames',varNames)

%% plot surface and generate gif
% plot ISE space
s=tf('s');
K=1;
tau=1;
%L=10;

if L<1 %big searching space(Kp,,ki>=100 for L<0.1)
    kp=1:1:15; ki=1:15/length(kp):15;
elseif L>=1 && L<10 %average searching space (kp,ki<1 for L>10)
    kp=0.1:0.1:1; ki=0.1:1/length(kp):1;%+1/length(kp);
elseif L>=10 %small searching space (kp,ki<1 for L>10)
    kp=0.01:0.01:0.12; ki=0.01:0.01:0.12;
end

p=((K*exp(-L*s))/(tau*s+1));
t=0:0.05:300;
refAmplitude=50;
LS=30*ones(1,round(length(t)/2)-1); %high state
HS=50*ones(1,round(length(t)/2));    %low state
R=[LS HS];                        %trajectory
plot(t,R)

%% calculate ISE plot

figure()
for i=1:length(kp)
    for j=1:length(ki)
        
        c=kp(i)+ki(j)/s;
        T=feedback(c*p,1);
        Y=lsim(T,R,t);
        
%         OV(i,j)=abs(max(Y)-refAmplitude);
        OV(i,j)=(abs(max(Y)-refAmplitude)/refAmplitude)*100;
        
%         ts(i,j)=settlingTime(Y(1:round(length(Y)/2)),15,t);
        ts(i,j)=converTimeInvReal(Y,10,0.5,refAmplitude);
        
        
        ise(i,j)=  sum ( (Y(round(length(Y)/2):end)'-R(round(length(Y)/2):end)).^2  );
        
        
        if L<1 %big searching space(Kp,,ki>=100 for L<0.1)
            J2(i,j)=(5*OV(i,j)+(0.1)*ise(i,j)+(1)*ts(i,j))/1.5e1 -45;
        elseif L>=1 && L<10 %average searching space (kp,ki<1 for L>10)
            J2(i,j)=(5*OV(i,j)+(0.1)*ise(i,j)+(1)*ts(i,j))/1.5e1- 40;
        elseif L>=10 %small searching space (kp,ki<1 for L>10)
            J2(i,j)=(5*OV(i,j)+(0.1)*ise(i,j)+(1)*ts(i,j))/1.5e1 +100;
        end
        
      
     
    end
end

[KP KI]=meshgrid(kp,ki);
s1=surf(KP,KI,J2)
direction = [0 0 1];
rotate(s1,direction,-200)
xlabel('KP');ylabel('KI');zlabel('J')

%% made gif surface
fileName = strcat('C:\Users\jairo\Dropbox\doctorado UC\yangquan assignments\DT peltier\DT SOC\optimGCNML',string(L),'.gif'); % Specify the output file name
yNM=costJSub;
kpOsub=downsample(kpOptim,30001);
kpOsub=kpOsub(1:length(yNM));
kiOsub=downsample(kiOptim,30001);
kiOsub=kpOsub(1:length(yNM))
optimGifFreqTun(fileName,kpOsub,kiOsub,-yNM,J2,KP,KI,L);

%% make gif time Response
fileName2 = strcat('C:\Users\jairo\Dropbox\doctorado UC\yangquan assignments\DT peltier\DT SOC\optimGCNMLTime',string(L),'.gif'); % Specify the output file name
optimGifTimeResponse(fileName2,kpOsub,kiOsub,-yNM,J2,KP,KI,L,YZN,yOut,ref,time)




