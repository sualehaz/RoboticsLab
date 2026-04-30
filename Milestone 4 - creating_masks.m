function [redMask, blueMask, greenMask, yellowMask] = creating_masks()

[depth_img, rgb_img, depth_data] = depth_sensor();

%% 1. Read Image
imgRGB = rgb_img;
% figure; 
% imshow(imgRGB);
% title('Original RGB Image');

%% 2. Convert to LAB
imgLab = rgb2lab(imgRGB); % Switch to LAB space to make color separation easier
[L,a,b] = imsplit(imgLab); % Break it down into lightness and color channels
% figure;
% imshow(imgLab);
% title ('LAB image');

%% 3. Segment Yellow Block
yellowMask = b > 30; % Isolating yellow using the 'b' channel (blue-yellow)
yellowMask = bwareaopen(yellowMask, 200); % Getting rid of small noise or "speckles"
% figure;
% imshow(yellowMask);
% title ('yellow mask applied');

%% 3. Segment Blue Block
blueMask = b < -34; % Isolating yellow using the 'b' channel (blue-yellow)
blueMask = bwareaopen(blueMask, 200); % Getting rid of small noise or "speckles"
% figure;
% imshow(blueMask);
% title ('blue mask applied');

%% 3. Segment Red Block
redMask = a > 25; % Isolating yellow using the 'b' channel (blue-yellow)
redMask = bwareaopen(redMask, 200); % Getting rid of small noise or "speckles"
figure;
imshow(redMask);
title ('red mask applied');

%% 3. Segment Green Block
greenMask = a < -13; % Isolating yellow using the 'b' channel (blue-yellow)
greenMask = bwareaopen(greenMask, 200); % Getting rid of small noise or "speckles"
% figure;
% imshow(greenMask);
% title ('green mask applied');


