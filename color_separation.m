function [graymap,redmap,greenmap,bluemap] = color_separation(pix)
%Input arguments: pix = RGB picture

%Output arguments:
%graymap = grayscale image
%redmap = red image plane
%greenmap = green image plane
%bluemap = blue image plane
 
mono = rgb2gray(pix);
 
graymap = pix;
graymap(:,:,1) = mono;
graymap(:,:,2) = mono;
graymap(:,:,3) = mono;
 
redmap = pix;
redmap(:,:,2) = 0;
redmap(:,:,3) = 0;
 
greenmap = pix;
greenmap(:,:,1) = 0;
greenmap(:,:,3) = 0;
 
bluemap = pix;
bluemap(:,:,1) = 0;
bluemap(:,:,2) = 0;
 
image([graymap,redmap;greenmap,bluemap]);