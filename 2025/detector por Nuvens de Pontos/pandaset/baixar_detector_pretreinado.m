outputFolder = fullfile(tempdir,'Pandaset');
lidarURL = ['https://ssd.mathworks.com/supportfiles/lidar/data/' ...
    'Pandaset_LidarData.tar.gz'];
helperDownloadPandasetData(outputFolder, lidarURL);
