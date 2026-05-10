
[depth_img, imgRGB, depth_data] = depth_sensor();
[redMask, blueMask, greenMask, yellowMask] = creating_masks(imgRGB);

Mask = redMask;

[labeledImage, numObjects] = bwlabel(Mask);

allBlockDepths = zeros(1, numObjects);

 for i = 1:numObjects
    objectMask = (labeledImage == i);
    Stats = regionprops(objectMask, 'BoundingBox', 'Centroid', 'Area');
    maskedDepth = double(depth_img) .* double(objectMask);
    thisDepth = max(maskedDepth(:)) * 100; 
    allBlockDepths(i) = thisDepth;
    fprintf('Block %d depth: %.2f mm\n', i, thisDepth);
 end

%{
height from camera to board = 45 (approx)
block depth = 45 - block height

case 1: 2 red square blocks + 1 yellow rectangle block
- block depth = 45 - 15 = 30
- test 1: 38.6584  ( +x, y = 0)
- test 2: 39.1208  ( x = 0, -y)
- test 3: 37.4335  ( x = 0, +y)
- test 4: 39.2333  ( -x, y = 0) (stacked them differently) 

block heights:
square: 3.3 cm
rectangle: 5 cm
rectangle: 2.6 cm 

%}

%{
Algorithm for stacking 

if block depth < 3.3 or < 5 or < 2.6
    recognize color
    pick and place them in designated spot 

else
   if height > 5 
      apply mask
      recognize color 
      place in position
      check height again
%}