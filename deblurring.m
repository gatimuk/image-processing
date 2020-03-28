load('blur_data.mat');
% colormap(photo_gray_map);
colormap('gray');
L = 17;
N = 300;
H = toeplitz([1/L;zeros(N-1,1)],[1/L*ones(1,L),zeros(1,N-L)]);
% Deconvolve the image
X = blur_photo/H;
% Display the image
imagesc(X);