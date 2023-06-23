url = '1.png';
I = imread(url);
imshow(I);

% Use ginput to select points
disp('Select the first point:');
[x1, y1] = ginput(1);
disp('Select the second point:');
[x2, y2] = ginput(1);

% Convert the coordinates to integers
x1 = round(x1);
y1 = round(y1);
x2 = round(x2);
y2 = round(y2);

% Display the selected coordinates
disp(['First point: (' num2str(x1) ', ' num2str(y1) ')']);
disp(['Second point: (' num2str(x2) ', ' num2str(y2) ')']);


% Convert the image to grayscale
gray = rgb2gray(I);

% Perform thresholding to obtain a binary image
threshold = 128;
binary_image = gray < threshold;

% Perform dilation on the binary image
se = strel('square', 7);
walls = imdilate(binary_image, se);

imshow(walls)
hold on;

D1 = bwdistgeodesic(~walls, x1, y1, 'quasi-euclidean');
D2 = bwdistgeodesic(~walls, x2, y2, 'quasi-euclidean');

D = D1 + D2;
D = round(D * 8) / 8;

D(isnan(D)) = inf;
paths = imregionalmin(D);

solution_path = bwmorph(paths, 'thin', inf);
thick_solution_path = imdilate(solution_path, ones(3,3));
P = imoverlay(I, thick_solution_path, [1 0 0]);
imshow(P, 'InitialMagnification', 'fit')

