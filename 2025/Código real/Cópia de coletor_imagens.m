j=1;
realSense = realsenseSubscriberSO_ARM;
images = {};
pClouds = {};
%%
while(1)
    [rgbImg, depthImg] = realSense.step;
    
    imwrite(rgbImg,strcat(num2str(j),'calib.png'),'png')
    imwrite(depthImg(:,:,3),strcat(num2str(j),'_d.png'),'png')
    images{j}=rgbImg;
    pClouds{j}=depthImg;
    
    j= j+1;
    pause(1);
end 
%%
save('ARM_2025_Calibration','images')