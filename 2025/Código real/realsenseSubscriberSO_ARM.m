classdef realsenseSubscriberSO_ARM < matlab.System
    % realsenseSubscriberSO System object to read color images and pointclouds from D415 RealSense camera.

    properties (Access = private)
        % Private properties
        pipeline; % RealSense pipeline object
        width = 640; % Image width
        height = 480; % Image height
        pointcloud; % Pointcloud
    end

    properties (Nontunable)
        FilePath = "C:\Program Files (x86)\Intel RealSense SDK 2.0\matlab"
    end

    methods (Access = protected)
        function setupImpl(obj)

            %Include perception MATLAB wrapper to path
            addpath(obj.FilePath);

            % Initialize the RealSense camera
            % Create a pipeline object
            obj.pipeline = realsense.pipeline();

            % Initialize the pointcloud
            obj.pointcloud = realsense.pointcloud();

            % Create a config object and enable the streams
            config = realsense.config();
            config.enable_stream(realsense.stream.color, obj.width, obj.height, realsense.format.rgb8, 15); % 15 in fps
            config.enable_stream(realsense.stream.depth, obj.width, obj.height, realsense.format.z16, 15);

            % Start streaming
            obj.pipeline.start(config);
            
            % Remove auto-exposure configs
            % device = obj.pipeline.get_active_profile().get_device();
            % color_sensor = device.first.color_sensor();% Turn off auto-exposure
            % color_sensor.set_option(realsense.option.enable_auto_exposure, false); % Turn off auto white balance
            % color_sensor.set_option(realsense.option.enable_auto_white_balance, false);
            
            % ignore first few frames
            for i=1:10
                obj.pipeline.wait_for_frames();
            end
        end

        function [colorImage, orderedPointCloud] = stepImpl(obj)
            % Output the color image and point cloud from the RealSense camera

            % Wait for the next set of frames -- blocking**
            frameset = obj.pipeline.wait_for_frames();

            % Get the color frame
            colorFrame = frameset.get_color_frame();
            colorImage = colorFrame.get_data();
            % Column vector to color matrix.
            colorImage = permute(reshape(colorImage', [3, colorFrame.get_width(), colorFrame.get_height()]), [3, 2, 1]);

            % Map the pointcloud to the color frame -- req. for aligning
            % depth to color image
            obj.pointcloud.map_to(colorFrame);

            % Get the depth frame
            depth_frame = frameset.get_depth_frame();
            points = obj.pointcloud.calculate(depth_frame);

            % Generate the pointcloud from the vertices
            vertices = points.get_vertices();
            % Column vector to color matrix.
            orderedPointCloud = permute(reshape(vertices, obj.width, obj.height, []), [2,1,3]);
        end

        function releaseImpl(obj)
            % Release resources
            obj.pipeline.stop();
        end

        % Propagation methods
        function [flag1, flag2] = isOutputFixedSizeImpl(~)
            % Indicate that the output sizes are fixed
            flag1 = true; % Color image
            flag2 = true; % Point cloud
        end

        function [size1, size2] = getOutputSizeImpl(obj)
            % Return the sizes of the outputs
            size1 = [obj.height, obj.width, 3]; % Color image
            size2 = [obj.height, obj.width, 3]; % Point cloud (X, Y, Z)
        end

        function [dataType1, dataType2] = getOutputDataTypeImpl(~)
            % Return the data types of the outputs
            dataType1 = 'uint8'; % Color image
            dataType2 = 'double'; % Point cloud
        end

        function [flag1, flag2] = isOutputComplexImpl(~)
            % Indicate that the outputs are not complex
            flag1 = false; % Color image
            flag2 = false; % Point cloud
        end
    end

    methods(Static, Access = protected)
        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog
            simMode = "Interpreted execution";
        end

        function flag = showSimulateUsingImpl
            % Return false if simulation mode hidden in System block dialog
            flag = false;
        end
    end
end