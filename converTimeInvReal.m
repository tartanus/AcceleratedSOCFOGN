function CT=converTimeInvReal(yin,sw,epsilon,refAmplitude)
%     sw=30;           %sliding window similar size to the reset window
%     epsilon= 1e-8;   %tolerance to detect convergence
    y = downsample(yin,1);    

    yStd=zeros(1,length(y));
    CT=0;
    counter=1;
    for i=length(y)-sw:-1:1
        yStd(i)= abs(refAmplitude-(mean(y(i:i+sw))));
%         yStd(isnan(yStd))=0;
%         aux=
        if(yStd(i)>epsilon)
            CT=i*1;     %adjust downsampling ratio
%             CT=i;     %adjust downsampling ratio
          break
        else
            CT=0*length(y);
        end
        counter=counter+1;
    end
 CT;
end