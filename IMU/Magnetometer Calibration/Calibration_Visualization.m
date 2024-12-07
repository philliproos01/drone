% Read data from the first file
fileID1 = fopen('mag_data_uncalibrated.txt', 'r');
if fileID1 == -1
    error('File newmag.txt could not be opened');
end
data1 = textscan(fileID1, '%f %f %f', 'Delimiter', '\t');
fclose(fileID1);

% Extract columns from the first file
x1 = data1{1};
y1 = data1{2};
z1 = data1{3};

% Read data from the second file
fileID2 = fopen('mag_data_calibrated.txt', 'r');
if fileID2 == -1
    error('File newmag2.txt could not be opened');
end
data2 = textscan(fileID2, '%f %f %f', 'Delimiter', '\t');
fclose(fileID2);

% Extract columns from the second file
x2 = data2{1};
y2 = data2{2};
z2 = data2{3};

% Create a 3D scatter plot
figure;
scatter3(x1, y1, z1, 'filled', 'blue');
hold on;
scatter3(x2, y2, z2, 'filled', 'red');
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('3D Plot of Data from Two Files');
legend('newmag.txt', 'newmag2.txt');
grid on;
