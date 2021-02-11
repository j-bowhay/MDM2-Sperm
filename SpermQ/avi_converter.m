obj = VideoReader('obj 3 phase 20x dia 92 1x zoom no binning Rabbit delta 78 heterozygote022.avi');
vid = read(obj);
frames = obj.NumFrames;

for x = 1 : frames
    imwrite(vid(:,:,:,x),strcat('frame- ',num2str(1),'.tif'), 'Compression', 'none', 'WriteMode', 'append');
end
